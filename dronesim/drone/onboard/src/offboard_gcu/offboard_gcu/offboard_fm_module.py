#!/usr/bin/python3

# python implementation of px4 multi-vehicle offboard control
# load public libraries
import os
import time
import numpy
import math

# load ROS2 related libraries
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
# from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_inverse

# load ROS2 messages
from px4_msgs.msg import *

# load private libraries and messages
from .offboard_library import flight_management, get_projection_matrix

# set publisher and subscriber quality of service profile
qos_profile_pub = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
qos_profile_sub = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
    history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# define offboard mission module using class
class offboard_mission_module(rclpy.node.Node,flight_management):

    def __init__(self):

        # inherit from the parent class 'rclpy.node.Node'
        super().__init__('OffboardControl')

        # load parameters from the parent class 'flight_management'
        super().load_parameters()

        # set PX4 instance
        PX4_NS  =   []
        fmu     =   []

        for idx in range(self.num_drones):
            PX4_NS.insert(idx,os.getenv("PX4_MICRODDS_NS_"+str(idx+1)))
            fmu.insert(idx,f"{PX4_NS[idx]}")

        # define publishers
        self.vehicle_command_publisher_             =   []
        self.trajectory_setpoint_publisher_         =   []
        self.offboard_control_mode_publisher_       =   []

        for idx in range(self.num_drones):
            self.vehicle_command_publisher_.insert(idx,self.create_publisher(VehicleCommand,f'/{fmu[idx]}/fmu/in/vehicle_command',qos_profile_pub))
            self.trajectory_setpoint_publisher_.insert(idx,self.create_publisher(TrajectorySetpoint,f'/{fmu[idx]}/fmu/in/trajectory_setpoint', qos_profile_pub))
            self.offboard_control_mode_publisher_.insert(idx,self.create_publisher(OffboardControlMode,f'/{fmu[idx]}/fmu/in/offboard_control_mode',qos_profile_pub))

        # define subscribers
        self.vehicle_status_subscriber_             =   []
        self.takeoff_status_subscriber_             =   []
        self.vehicle_local_position_subscriber_     =   []
        self.vehicle_odometry_subscriber_           =   []

        for idx in range(self.num_drones):
            self.vehicle_status_subscriber_.insert(idx,self.create_subscription(VehicleStatus,f'/{fmu[idx]}/fmu/out/vehicle_status',lambda msg:self.process_vehicle_status(msg,idx),qos_profile_sub))
            self.takeoff_status_subscriber_.insert(idx,self.create_subscription(TakeoffStatus,f'/{fmu[idx]}/fmu/out/takeoff_status',lambda msg:self.process_takeoff_status(msg,idx),qos_profile_sub))
            self.vehicle_local_position_subscriber_.insert(idx,self.create_subscription(VehicleLocalPosition,f'/{fmu[idx]}/fmu/out/vehicle_local_position',lambda msg:self.process_vehicle_local_position(msg,idx),qos_profile_sub))
            self.vehicle_odometry_subscriber_.insert(idx,self.create_subscription(VehicleOdometry,f'{fmu[idx]}/out/vehicle_odometry',lambda msg:self.process_vehicle_odometry,qos_profile_sub))

        # initialize parameters for timer callback function
        self.timer_period                           =   numpy.float32(0.1)  # 100 milliseconds
        self.timer_                                 =   self.create_timer(self.timer_period,self.obfm_callback)

        self.temp_callback_counter_                 =   []
        self.total_callback_counter_                =   []

        for idx in range(self.num_drones):
            self.temp_callback_counter_.insert(idx,numpy.uint64(0))
            self.total_callback_counter_.insert(idx,numpy.uint64(0))

        # initialize parameters for flight management module
        self.flight_phase_          =   []
        self.entry_execute_         =   []

        for idx in range(self.num_drones):
            self.flight_phase_.insert(idx,numpy.uint8(0))
            self.entry_execute_.insert(idx,numpy.uint8(1))

        # initialize parameters for offboard wpt mission
        self.past_setpoint_         =   []
        self.true_setpoint_         =   []
        self.atck_setpoint_         =   []

        self.atck_engage_           =   []
        self.atck_detect_           =   []

        self.atck_act_states_       =   []
        self.atck_est_states_       =   []
        self.mock_meas_noises_      =   []

        for idx in range(self.num_drones):
            self.past_setpoint_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))
            self.true_setpoint_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))
            self.atck_setpoint_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))

            self.atck_engage_.insert(idx,False)
            self.atck_detect_.insert(idx,True)

            self.atck_act_states_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))
            self.atck_est_states_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))
            self.mock_meas_noises_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))

        # initialize fields for trajectory setpoint topic
        self.trajectory_setpoint_x_     =   []
        self.trajectory_setpoint_y_     =   []
        self.trajectory_setpoint_z_     =   []
        self.trajectory_setpoint_yaw_   =   []

        for idx in range(self.num_drones):
            self.trajectory_setpoint_x_.insert(idx,numpy.float32(0.0))
            self.trajectory_setpoint_y_.insert(idx,numpy.float32(0.0))
            self.trajectory_setpoint_z_.insert(idx,numpy.float32(0.0))
            self.trajectory_setpoint_yaw_.insert(idx,numpy.float32(0.0))

        # initialize fields of offboard control mode topic
        self.offboard_ctrl_position_        =   []
        self.offboard_ctrl_position_        =   []
        self.offboard_ctrl_velocity_        =   []
        self.offboard_ctrl_acceleration_    =   []
        self.offboard_ctrl_attitude_        =   []
        self.offboard_ctrl_body_rate_       =   []
        self.offboard_ctrl_actuator_        =   []

        for idx in range(self.num_drones):
            self.offboard_ctrl_position_.insert(idx,False)
            self.offboard_ctrl_velocity_.insert(idx,False)
            self.offboard_ctrl_acceleration_.insert(idx,False)
            self.offboard_ctrl_attitude_.insert(idx,False)
            self.offboard_ctrl_body_rate_.insert(idx,False)
            self.offboard_ctrl_actuator_.insert(idx,False)

        # initialize fields of vehicle status topic
        self.status_nav_state_              =   []
        self.status_arming_status_          =   []

        for idx in range(self.num_drones):
            self.status_nav_state_.insert(idx,numpy.uint8(0))
            self.status_arming_status_.insert(idx,numpy.uint8(0))

        # initialize a field of takeoff status topic
        # self.status_takeoff_                =   []

        # for idx in range(self.num_drones):
            # self.status_takeoff_.insert(idx,numpy.uint8(0))

        # initialize fields of local position topic
        self.vehicle_local_position_ned_    =   []
        self.vehicle_local_velocity_ned_    =   []

        for idx in range(self.num_drones):
            self.vehicle_local_position_ned_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))
            self.vehicle_local_velocity_ned_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))

        # initialize fields of vehicle odometry topic
        self.vehicle_odometry_position_     =   []
        self.vehicle_odometry_velocity_     =   []
        self.vehicle_odometry_quaternion_   =   []

        for idx in range(self.num_drones):
            self.vehicle_odometry_position_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))
            self.vehicle_odometry_velocity_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float32))
            self.vehicle_odometry_quaternion_.insert(idx,numpy.array([0.0,0.0,0.0,0.0],dtype=numpy.float32))

        # declare subscribers once to avoid unused variable warning
        self.vehicle_status_subscriber_
        # self.takeoff_status_subscriber_
        self.vehicle_local_position_subscriber_
        self.vehicle_odometry_subscriber_

    # offboard flight management callback function
    def obfm_callback(self):

        #------------------- Offboard control test code -------------------
        # for idx in range(self.num_drones):
        #     if (self.total_callback_counter_ <= numpy.uint64(10/self.timer_period)):
        #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1, \
        #             param2=self.PX4_CUSTOM_MAIN_MODE_MANUAL_,idx=idx)

        #     elif (self.total_callback_counter_ > numpy.uint64(10/self.timer_period)) \
        #         and (self.total_callback_counter_ <= numpy.uint64(30/self.timer_period)):
        #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=self.ARMING_ACTION_ARM_,idx=idx)
        #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1, \
        #             param2=self.PX4_CUSTOM_MAIN_MODE_AUTO_,param3=self.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF_,idx=idx)

        #     elif (self.total_callback_counter_ > numpy.uint64(30/self.timer_period)):
        #         self.trajectory_setpoint_z_[idx]    =	numpy.float32(-10.0)
        #         self.publish_trajectory_setpoint(idx=idx)
        #         self.offboard_ctrl_position_[idx]   = True
        #         self.publish_offboard_control_mode(idx=idx)
        #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_OFFBOARD_,idx=idx)

        #     else:
        #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_OFFBOARD_,idx=idx)

        #     self.temp_callback_counter_[idx]            =   self.temp_callback_counter_[idx]+1
        #     self.total_callback_counter_[idx]           =   self.total_callback_counter_[idx]+1

        # ------------------- mission flight using offboard control -------------------
        # create a log message at NuttShell
        # self.get_logger().info('Offboard Mission Flight Manager Callback')

        # for testing the developed spoofing attack detector
        # need to be revised when the code is built and integrated in the real hardware

        for idx in range(self.num_drones):

            # phase 0: manual(idle) - switch to manual mode (from any mode of last missions)
            # set PX4_CUSTOM_MAIN_MODE to PX4_CUSTOM_MAIN_MODE_MANUAL and hold this mode for 10 seconds
            if self.flight_phase_[idx] == 0:

                # entry:
                if self.entry_execute_[idx]:

                    self.entry_execute_[idx] 	        = 	numpy.uint64(0)
                    self.temp_callback_counter_[idx]    =   numpy.uint64(0)
                    self.total_callback_counter_[idx]   =   numpy.uint64(0)

                # during:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_MANUAL_,idx=idx)
                self.get_logger().info("Current Mode: Manual (Idle)")

                # transition:
                if (self.status_nav_state_[idx] == self.NAVIGATION_STATE_MANUAL_) and (self.temp_callback_counter_[idx] > numpy.uint64(10/self.timer_period)):

                    # exit:
                    self.flight_phase_[idx]     =	1
                    self.entry_execute_[idx]    =	1

            # phase 1: auto-takeoff - switch to auto/takeoff mode
            # arm the multicopter by setting VEHICLE_CMD_COMPONENT_ARM_DISARM to ARMING_ACTION_ARM
            # engage auto mode and takeoff submode
            elif self.flight_phase_[idx] == 1:

                # entry:
                if self.entry_execute_[idx]:

                    self.entry_execute_[idx] 	        = 	numpy.uint64(0)
                    self.temp_callback_counter_[idx]    =   numpy.uint64(0)

                # during:
                if (self.status_arming_status_[idx] == self.ARMING_STATE_STANDBY_):
                    # debug: arming -> [tone_alarm] notify negative
                    # debug: cannot subscribe takeoff status
                    # and (self.status_takeoff_ <= self.TAKEOFF_STATE_READY_FOR_TAKEOFF_): -> removed
                    # debug: status_take directly becomes 5 from 0 when arming is engaged
                    # need to be checked whether this originates from the low communication rate between
                    # ROS2 - PX4 or inherent setting of this module (or might be bug)
                    # \ and (self.status_takeoff_ <= self.TAKEOFF_STATE_FLIGHT_): -> removed
                    # this might also be related with specifying trajectory setpoint
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_AUTO_, \
                        param3=self.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF_,idx=idx)
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=self.ARMING_ACTION_ARM_,idx=idx)
                    self.get_logger().info("Current Mode: TakeOff (Arm)")

                elif (self.status_arming_status_[idx] == self.ARMING_STATE_ARMED_):
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_AUTO_, \
                        param3=self.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF_,idx=idx)
                    self.get_logger().info("Current Mode: TakeOff (Auto-TakeOff)")

                # transition:
                # debug: when the multirotor lands on the point which deviates from the original launch
                # point, the local position subscriber is unable to subscribe the local position topic
                # possibly, originates from the initialization problem of EKF2
                # debug: cannot subscribe takeoff status
                # (self.status_takeoff_ == self.TAKEOFF_STATE_FLIGHT_) and
                if (self.vehicle_local_position_ned_[idx][2] < -numpy.float32(2.0)):

                    # exit:
                    self.flight_phase_[idx]     =	2
                    self.entry_execute_[idx]    =	1

            # phase 2: engage offboard control - switch to offboard/position mode
            # hold its position at a starting point
            # proceed to offboard wpt mission when the multicoptor reaches a setpoint
            elif self.flight_phase_[idx] == 2:

                # entry:
                if self.entry_execute_[idx]:

                    self.entry_execute_[idx] 			= 	0
                    self.temp_callback_counter_[idx]    =   0
                    self.offboard_ctrl_position_[idx]   =	True
                    self.trajectory_setpoint_x_[idx]    =   0
                    self.trajectory_setpoint_y_[idx]    =   0
                    self.trajectory_setpoint_z_[idx]    =   numpy.float32(-2.5)
                    self.trajectory_setpoint_yaw_[idx]  =   0

                # during:
                self.get_logger().info("Current Mode: Offboard (Position hold at a starting point)")

                self.publish_trajectory_setpoint(idx=idx)
                self.publish_offboard_control_mode(idx=idx)

                if (self.status_nav_state_[idx] != self.NAVIGATION_STATE_OFFBOARD_):
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1, \
                        param2=self.PX4_CUSTOM_MAIN_MODE_OFFBOARD_,idx=idx)

                # transition
                dist_xyz    =   numpy.sqrt(numpy.power(self.trajectory_setpoint_x_[idx]- \
                                    self.vehicle_local_position_ned_[idx][0],2)+numpy.power( \
                                    self.trajectory_setpoint_y_[idx]-self.vehicle_local_position_ned_[idx][1],2)+ \
                                    numpy.power(self.trajectory_setpoint_z_[idx]-self.vehicle_local_position_ned_[idx][2],2))

                if dist_xyz < self.nav_wpt_reach_rad_:

                    # exit:
                    self.flight_phase_[idx]     =	3
                    self.entry_execute_[idx]    =	1

            # phase 3: engage offboard wpt mission - hold offboard/position mode
            # perform a waypoint mission and com
            # exit when the multicoptor reaches a last waypoint and the autopilot turns off
            elif self.flight_phase_[idx] == 3:

                # entry:
                if self.entry_execute_[idx]:

                    self.entry_execute_[idx] 			= 	0
                    self.temp_callback_counter_[idx]    =   0
                    self.past_setpoint_[idx]            =   [self.trajectory_setpoint_x_[idx],self.trajectory_setpoint_y_[idx],self.trajectory_setpoint_z_[idx]]
                    self.true_setpoint_[idx]            =   numpy.add(self.past_setpoint_[idx],numpy.array([10.0,0.0,0.0],dtype=numpy.float32))
                    self.atck_setpoint_[idx]            =   numpy.add(self.past_setpoint_[idx],numpy.array([10.0,4.0,0.0],dtype=numpy.float32))
                    self.atck_engage_[idx]              =   True
                    self.atck_detect_[idx]              =   False

                # during:
                alpha       =   (self.vehicle_local_position_ned_[idx][0]-self.past_setpoint_[idx][0])/ \
                                (self.true_setpoint_[idx][0]-self.past_setpoint_[idx][0])
                alpha       =   numpy.clip(2*alpha,a_min = 0,a_max = 1)

                if self.atck_engage_[idx] and not self.atck_detect_[idx]:
                    alpha   =   alpha
                else:
                    alpha   =   0

                self.trajectory_setpoint_x_[idx]    =   (1-alpha)*self.true_setpoint_[idx][0]+alpha*self.atck_setpoint_[idx][0]
                self.trajectory_setpoint_y_[idx]    =   (1-alpha)*self.true_setpoint_[idx][1]+alpha*self.atck_setpoint_[idx][1]
                self.trajectory_setpoint_z_[idx]    =   (1-alpha)*self.true_setpoint_[idx][2]+alpha*self.atck_setpoint_[idx][2]

                self.publish_offboard_control_mode(idx=idx)
                self.publish_trajectory_setpoint(idx=idx)

                proj_atck   =   get_projection_matrix(self.true_setpoint_[idx]-self.atck_setpoint_[idx])

                self.atck_act_states_[idx]  =   self.vehicle_local_position_ned_[idx]
                ack_vec                     =   (numpy.matmul(numpy.atleast_2d(self.atck_act_states_[idx]),proj_atck)).flatten()

                self.atck_est_states_[idx]  =   (numpy.matmul(numpy.atleast_2d(self.atck_est_states_[idx]),(numpy.eye(3,dtype=numpy.float32)-self.Lobv).T)).flatten()+ \
                                                (numpy.matmul(numpy.atleast_2d(self.vehicle_local_velocity_ned_[idx]),(numpy.eye(3,dtype=numpy.float32)-proj_atck).T)).flatten()*self.timer_period+ \
                                                (numpy.matmul(numpy.atleast_2d(self.atck_act_states_[idx]),(self.Lobv).T)).flatten()- \
                                                (numpy.matmul(numpy.atleast_2d(ack_vec),(self.Lobv).T)).flatten()

                if numpy.linalg.norm(self.atck_est_states_[idx]-self.atck_act_states_[idx]) >= self.detect_threshold:
                    self.atck_engage_[idx]              =   False
                    self.atck_detect_[idx]              =   True

                # transition
                dist_xyz    =   numpy.sqrt(numpy.power(self.true_setpoint_[idx][0]- \
                                    self.vehicle_local_position_ned_[idx][0],2)+numpy.power( \
                                    self.true_setpoint_[idx][1]-self.vehicle_local_position_ned_[idx][1],2)+ \
                                    numpy.power(self.true_setpoint_[idx][2]-self.vehicle_local_position_ned_[idx][2],2))

                if dist_xyz < self.nav_wpt_reach_rad_:

                    # exit:
                    self.flight_phase_[idx]     =	4
                    self.entry_execute_[idx]    =	1

            # phase 4: return to launch mode
            elif self.flight_phase_[idx] == 4:

                # entry:
                if self.entry_execute_[idx]:

                    self.entry_execute_[idx] 			= 	0
                    self.temp_callback_counter_[idx]    =   0

                # during:
                self.get_logger().info("Current Mode: Return to Launch")

                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1, \
                        param2=self.PX4_CUSTOM_MAIN_MODE_AUTO_, \
                        param3=self.PX4_CUSTOM_SUB_MODE_AUTO_RTL_,idx=idx)

                # transition:
                if 0:

                    # exit:
                    self.flight_phase_          =	5
                    self.entry_execute_         =	1

            else:

                # during:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_MANUAL_,idx=idx)
                self.get_logger().info("Current Mode: Manual (Idle)")

            self.temp_callback_counter_[idx]    =   self.temp_callback_counter_[idx]+1
            self.total_callback_counter_[idx]   =   self.total_callback_counter_[idx]+1

    # publisher functions
    def publish_vehicle_command(self,command,param1=float(0.0),param2=float(0.0),param3=float(0.0),idx=numpy.uint8(0)):
        msg                                         =   VehicleCommand()
        msg.timestamp                               =   int(rclpy.clock.Clock().now().nanoseconds/1000) # time in microseconds
        msg.param1                                  =   float(param1)
        msg.param2                                  =   float(param2)
        msg.param3                                  =   float(param3)
        msg.command                                 =   command # command ID
        msg.target_system                           =   idx+2 # system which should execute the command
        msg.target_component                        =   1 # component which should execute the command, 0 for all components
        msg.source_system                           =   idx+2 # system sending the command
        msg.source_component                        =   1 # component sending the command
        msg.from_external                           =   True

        self.vehicle_command_publisher_[idx].publish(msg)

    def publish_trajectory_setpoint(self,idx=numpy.uint8(0)):
        msg                                         =   TrajectorySetpoint()
        msg.timestamp                               =   int(rclpy.clock.Clock().now().nanoseconds/1000) # time in microseconds
        msg.position[0]                             =   float(self.trajectory_setpoint_x_[idx])
        msg.position[1]                             =   float(self.trajectory_setpoint_y_[idx])
        msg.position[2]                             =   float(self.trajectory_setpoint_z_[idx])
        msg.yaw                                     =   float(self.trajectory_setpoint_yaw_[idx])

        self.trajectory_setpoint_publisher_[idx].publish(msg)

    def publish_offboard_control_mode(self,idx=numpy.uint8(0)):
        msg                                         =   OffboardControlMode()
        msg.timestamp                               =   int(rclpy.clock.Clock().now().nanoseconds/1000) # time in microseconds
        msg.position                                =   self.offboard_ctrl_position_[idx]
        msg.velocity                                =   self.offboard_ctrl_velocity_[idx]
        msg.acceleration                            =   self.offboard_ctrl_acceleration_[idx]
        msg.attitude                                =   self.offboard_ctrl_attitude_[idx]
        msg.body_rate                               =   self.offboard_ctrl_body_rate_[idx]

        self.offboard_control_mode_publisher_[idx].publish(msg)

    # Subscriber functions
    def process_vehicle_status(self,msg,idx=numpy.uint8(0)):
        self.status_nav_state_[idx]                 =   numpy.uint8(msg.nav_state)
        self.status_arming_status_[idx]             =   numpy.uint8(msg.arming_state)

    # def process_takeoff_status(self,msg,idx=numpy.uint8(0)):
        # self.status_takeoff_[idx]                   =   numpy.uint8(msg.takeoff_state)

    def process_vehicle_local_position(self,msg,idx=numpy.uint8(0)):
        self.vehicle_local_position_ned_[idx]       =   numpy.array([msg.x,msg.y,msg.z],dtype=numpy.float32)
        self.vehicle_local_velocity_ned_[idx]       =   numpy.array([msg.vx,msg.vy,msg.vz],dtype=numpy.float32)

    def process_vehicle_odometry(self,msg,idx=numpy.uint8(0)):
        self.vehicle_odometry_position_[idx]        =   numpy.array(msg.position,dtype=numpy.float32)
        self.vehicle_odometry_velocity_[idx]        =   numpy.array(msg.velocity,dtype=numpy.float32)
        self.vehicle_odometry_quaternion_[idx]      =   numpy.array(msg.q,dtype=numpy.float32)


def main(args = None):

    # initialize python rclpy library
    rclpy.init(args = args)

    print("Starting offboard control node...\n")
    offboard_control = offboard_mission_module()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()