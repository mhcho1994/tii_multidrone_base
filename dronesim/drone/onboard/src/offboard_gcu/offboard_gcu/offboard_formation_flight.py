#!/usr/bin/python3

# python implementation of px4 multi-vehicle offboard control
# load public libraries
import os
import time
import numpy
import math
import argparse

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

    def __init__(self,num_drones):

        # inherit from the parent class 'rclpy.node.Node'
        super().__init__('OffboardControl')

        # load parameters from the parent class 'flight_management'
        super().load_parameters()

        # set the number of drones
        self.num_drones     =   num_drones

        # set PX4 instance
        PX4_NS  =   []
        fmu     =   []

        for idx in range(self.num_drones):
            PX4_NS.insert(idx,os.getenv("PX4_MICRODDS_NS_"+str(idx+1)))
            fmu.insert(idx,f"{PX4_NS[idx]}")

        print(fmu)

        # define publishers
        self.vehicle_command_publisher_             =   []
        self.trajectory_setpoint_publisher_         =   []
        self.offboard_control_mode_publisher_       =   []

        for idx in range(self.num_drones):
            self.vehicle_command_publisher_.insert(idx,self.create_publisher(VehicleCommand,f'/{fmu[idx]}/fmu/in/vehicle_command',qos_profile_pub))
            self.trajectory_setpoint_publisher_.insert(idx,self.create_publisher(TrajectorySetpoint,f'/{fmu[idx]}/fmu/in/trajectory_setpoint', qos_profile_pub))
            self.offboard_control_mode_publisher_.insert(idx,self.create_publisher(OffboardControlMode,f'/{fmu[idx]}/fmu/in/offboard_control_mode',qos_profile_pub))

        # define subscribers
        # self.vehicle_status_subscriber_             =   []
        # self.vehicle_local_position_subscriber_     =   []

        # for idx in range(self.num_drones):
        #     self.vehicle_status_subscriber_.insert(idx,self.create_subscription(VehicleStatus,f'/{fmu[idx]}/fmu/out/vehicle_status',lambda msg:self.process_vehicle_status(msg,idx),qos_profile_sub))
        #     self.vehicle_local_position_subscriber_.insert(idx,self.create_subscription(VehicleLocalPosition,f'/{fmu[idx]}/fmu/out/vehicle_local_position',lambda msg:self.process_vehicle_local_position(msg,idx),qos_profile_sub))

        self.vehicle_status_subscriber1_ =  self.create_subscription(VehicleStatus,f'/{fmu[0]}/fmu/out/vehicle_status',lambda msg:self.process_vehicle_status(msg,0),qos_profile_sub)
        self.vehicle_status_subscriber2_ =  self.create_subscription(VehicleStatus,f'/{fmu[1]}/fmu/out/vehicle_status',lambda msg:self.process_vehicle_status(msg,1),qos_profile_sub)
        self.vehicle_status_subscriber3_ =  self.create_subscription(VehicleStatus,f'/{fmu[2]}/fmu/out/vehicle_status',lambda msg:self.process_vehicle_status(msg,2),qos_profile_sub)
        self.vehicle_status_subscriber4_ =  self.create_subscription(VehicleStatus,f'/{fmu[3]}/fmu/out/vehicle_status',lambda msg:self.process_vehicle_status(msg,3),qos_profile_sub)
        self.vehicle_status_subscriber5_ =  self.create_subscription(VehicleStatus,f'/{fmu[4]}/fmu/out/vehicle_status',lambda msg:self.process_vehicle_status(msg,4),qos_profile_sub)

        self.vehicle_local_position_subscriber1_    =   self.create_subscription(VehicleLocalPosition,f'/{fmu[0]}/fmu/out/vehicle_local_position',lambda msg:self.process_vehicle_local_position(msg,0),qos_profile_sub)
        self.vehicle_local_position_subscriber2_    =   self.create_subscription(VehicleLocalPosition,f'/{fmu[1]}/fmu/out/vehicle_local_position',lambda msg:self.process_vehicle_local_position(msg,1),qos_profile_sub)
        self.vehicle_local_position_subscriber3_    =   self.create_subscription(VehicleLocalPosition,f'/{fmu[2]}/fmu/out/vehicle_local_position',lambda msg:self.process_vehicle_local_position(msg,2),qos_profile_sub)
        self.vehicle_local_position_subscriber4_    =   self.create_subscription(VehicleLocalPosition,f'/{fmu[3]}/fmu/out/vehicle_local_position',lambda msg:self.process_vehicle_local_position(msg,3),qos_profile_sub)
        self.vehicle_local_position_subscriber5_    =   self.create_subscription(VehicleLocalPosition,f'/{fmu[4]}/fmu/out/vehicle_local_position',lambda msg:self.process_vehicle_local_position(msg,4),qos_profile_sub)

        # for idx in range(self.num_drones):
        #     globals()["self.vehicle_status_subscriber_{}".format(idx)]=self.create_subscription(VehicleStatus,f'/{fmu[idx]}/fmu/out/vehicle_status',lambda msg:self.process_vehicle_status(msg,idx),qos_profile_sub)
        #     globals()["self.vehicle_local_position_subscriber_{}".format(idx)]=self.create_subscription(VehicleLocalPosition,f'/{fmu[idx]}/fmu/out/vehicle_local_position',lambda msg:self.process_vehicle_local_position(msg,idx),qos_profile_sub)

        # initialize parameters for timer callback function
        self.timer_period                           =   numpy.float64(0.02)  # 20 milliseconds
        self.timer_                                 =   self.create_timer(self.timer_period,self.obfm_callback)

        self.temp_callback_counter_                 =   []
        self.total_callback_counter_                =   []

        for idx in range(self.num_drones):
            self.temp_callback_counter_.insert(idx,numpy.uint16(0))
            self.total_callback_counter_.insert(idx,numpy.uint16(0))

        # initialize parameters for flight management module
        self.flight_phase_          =   []
        self.entry_execute_         =   []

        for idx in range(self.num_drones):
            self.flight_phase_.insert(idx,numpy.uint8(0))
            self.entry_execute_.insert(idx,numpy.uint8(1))

        # initialize parameters for offboard formation flight mission
        self.past_wpt_              =   []
        self.cur_wpt_               =   []

        self.wpt_set_               =   numpy.array([[0.0,10.0,-10.0],
                                                     [5.0,15.0,-10.0],
                                                     [0.0,20.0,-10.0],
                                                     [-5.0,25.0,-10.0],
                                                     [0.0,30.0,-10.0]],dtype=numpy.float64)

        self.offset_                =   numpy.array([[-1.0,1.0,0.0],
                                                     [-1.0,-1.0,0.0],
                                                     [0.0,0.0,0.0],
                                                     [1.0,-1.0,0.0],
                                                     [1.0,1.0,0.0]],dtype=numpy.float64)

        self.theta_                 =   []
        self.omega_                 =   numpy.float64(0.1)
        self.wpt_idx_               =   []

        self.check                  =   [False,False,False,False,False]

        for idx in range(self.num_drones):
            self.past_wpt_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float64))
            self.cur_wpt_.insert(idx,numpy.array([0.0,0.0,0.0],dtype=numpy.float64))
            self.theta_.insert(idx,numpy.float64(0.0))
            self.wpt_idx_.insert(idx,numpy.int8(0))

        # initialize fields for trajectory setpoint topic
        self.trajectory_setpoint_x_     =   []
        self.trajectory_setpoint_y_     =   []
        self.trajectory_setpoint_z_     =   []
        self.trajectory_setpoint_yaw_   =   []

        for idx in range(self.num_drones):
            self.trajectory_setpoint_x_.insert(idx,numpy.float64(0.0))
            self.trajectory_setpoint_y_.insert(idx,numpy.float64(0.0))
            self.trajectory_setpoint_z_.insert(idx,numpy.float64(0.0))
            self.trajectory_setpoint_yaw_.insert(idx,numpy.float64(0.0))

        # initialize fields of offboard control mode topic
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

        # initialize fields of local position topic
        self.vehicle_local_position_ned_    =   []
        self.vehicle_local_velocity_ned_    =   []

        for idx in range(self.num_drones):
            self.vehicle_local_position_ned_.insert(idx,None)
            self.vehicle_local_velocity_ned_.insert(idx,None)

        # declare subscribers once to avoid unused variable warning
        # self.vehicle_status_subscriber_
        # self.vehicle_local_position_subscriber_

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


        # ------------------- Formation flight using offboard control -------------------
        # note: not exact, distributed control protocol will be included in later versions
        # create a log message at NuttShell
        # self.get_logger().info('Offboard Mission Flight Manager Callback')

        # for testing the developed distributed cyberattack mitigation algorithms
        # need to be revised when the code is built and integrated in the real hardware

        for idx in range(self.num_drones):
            
            # print("flight phase")
            # print(self.flight_phase_[idx])

            # phase 0: manual(idle) - switch to manual mode (from any mode of last missions)
            # set PX4_CUSTOM_MAIN_MODE to PX4_CUSTOM_MAIN_MODE_MANUAL and hold this mode for 10 seconds
            if self.flight_phase_[idx] == 0:

                # entry:
                if self.entry_execute_[idx]:

                    self.entry_execute_[idx] 	        = 	numpy.uint8(0)
                    self.temp_callback_counter_[idx]    =   numpy.uint16(0)
                    self.total_callback_counter_[idx]   =   numpy.uint16(0)

                # during:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_MANUAL_,idx=idx)
                self.get_logger().info('Current Mode: Manual (Idle) \ PX4 instance: '+str(idx))

                # transition:
                # print(idx)
                # print(self.status_nav_state_[idx] == self.NAVIGATION_STATE_MANUAL_)
                # print(self.temp_callback_counter_[idx] > numpy.uint64(10/self.timer_period))
                # print(self.vehicle_local_position_ned_[idx] is not None)

                if (self.status_nav_state_[idx] == self.NAVIGATION_STATE_MANUAL_) and (self.temp_callback_counter_[idx] > numpy.uint64(10/self.timer_period)) \
                    and (self.vehicle_local_position_ned_[idx] is not None) and (self.vehicle_local_velocity_ned_[idx] is not None):

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
                    self.get_logger().info('Current Mode: TakeOff (Arm) \ PX4 instance: '+str(idx))

                elif (self.status_arming_status_[idx] == self.ARMING_STATE_ARMED_):
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_AUTO_, \
                        param3=self.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF_,idx=idx)
                    self.get_logger().info('Current Mode: TakeOff (Auto-TakeOff) \ PX4 instance: '+str(idx))

                # transition:
                # debug: when the multirotor lands on the point which deviates from the original launch
                # point, the local position subscriber is unable to subscribe the local position topic
                # possibly, originates from the initialization problem of EKF2
                # debug: cannot subscribe takeoff status
                # (self.status_takeoff_ == self.TAKEOFF_STATE_FLIGHT_) and
                if (self.vehicle_local_position_ned_[idx][2] < numpy.float64(-2.0)):

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
                    self.trajectory_setpoint_x_[idx]    =   self.offset_[idx][0]
                    self.trajectory_setpoint_y_[idx]    =   self.offset_[idx][1]
                    self.trajectory_setpoint_z_[idx]    =   numpy.float64(-10.0)
                    self.trajectory_setpoint_yaw_[idx]  =   0

                # during:
                # self.get_logger().info("Current Mode: Offboard (Position hold at a starting point)")

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

                    self.check[idx]     =   True

                    # exit:
                    if all(self.check):
                        self.flight_phase_[idx]     =	3
                        self.entry_execute_[idx]    =	1

            # phase 3: engage offboard wpt mission - hold offboard/position mode
            # perform a waypoint mission and use a detector for spoofing attack
            # exit when the multicoptor reaches a last waypoint and the autopilot turns off
            elif self.flight_phase_[idx] == 3:

                # entry:
                if self.entry_execute_[idx]:

                    self.entry_execute_[idx] 			= 	0
                    self.temp_callback_counter_[idx]    =   0

                    self.theta_[idx]                    =   numpy.float64(0.0)
                    self.wpt_idx_[idx]                  =   numpy.uint8(0)
                    self.past_wpt_[idx]                 =   self.vehicle_local_position_ned_[idx]
                    self.cur_wpt_[idx]                  =   self.wpt_set_[self.wpt_idx_[idx]].flatten()+self.offset_[idx].flatten()

                # during:
                # publish offboard control modes
                self.offboard_ctrl_position_[idx] = True
                self.publish_offboard_control_mode(idx=idx)

                # publish offboard position cmd
                self.trajectory_setpoint_x_[idx] = self.theta_[idx]*self.cur_wpt_[idx][0]+(1-self.theta_[idx])*self.past_wpt_[idx][0]
                self.trajectory_setpoint_y_[idx] = self.theta_[idx]*self.cur_wpt_[idx][1]+(1-self.theta_[idx])*self.past_wpt_[idx][1]
                self.trajectory_setpoint_z_[idx] = self.theta_[idx]*self.cur_wpt_[idx][2]+(1-self.theta_[idx])*self.past_wpt_[idx][2]
                self.publish_trajectory_setpoint(idx=idx)

                if (self.status_nav_state_[idx] != self.NAVIGATION_STATE_OFFBOARD_):
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1, \
                        param2=self.PX4_CUSTOM_MAIN_MODE_OFFBOARD_,idx=idx)

                if (self.vehicle_local_position_ned_[idx] is not None) and (self.vehicle_local_velocity_ned_[idx] is not None):
                    dist_xyz    =   numpy.sqrt(numpy.power(self.cur_wpt_[idx][0]- \
                                        self.vehicle_local_position_ned_[idx][0],2)+numpy.power( \
                                        self.cur_wpt_[idx][1]-self.vehicle_local_position_ned_[idx][1],2)+ \
                                        numpy.power(self.cur_wpt_[idx][2]-self.vehicle_local_position_ned_[idx][2],2))

                    if (self.wpt_idx_[idx] <= 2) and (dist_xyz <= self.nav_wpt_reach_rad_):
                        self.theta_[idx]    = numpy.float64(0.0)
                        self.past_wpt_[idx] = self.cur_wpt_[idx].flatten()
                        self.wpt_idx_[idx]  = self.wpt_idx_[idx]+1
                        self.cur_wpt_[idx]  = self.wpt_set_[self.wpt_idx_[idx]].flatten()+self.offset_[idx].flatten()

                    self.theta_[idx]    =   self.theta_[idx]+self.omega_*self.timer_period
                    self.theta_[idx]    =   numpy.clip(self.theta_[idx],a_min=0.0,a_max=1.0)

                    # transition
                    if (self.wpt_idx_[idx] == 3) and (dist_xyz <= self.nav_wpt_reach_rad_):

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
                # self.get_logger().info("Current Mode: Return to Launch")

                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1, \
                        param2=self.PX4_CUSTOM_MAIN_MODE_AUTO_, \
                        param3=self.PX4_CUSTOM_SUB_MODE_AUTO_RTL_,idx=idx)

                # transition:
                if (self.vehicle_local_position_ned_[idx][2] < numpy.float64(-0.25)):

                    # exit:
                    print("Offboard mission finished")

            else:

                # during:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_MANUAL_,idx=idx)
                # self.get_logger().info("Current Mode: Manual (Idle)")

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

    def process_vehicle_local_position(self,msg,idx=numpy.uint8(0)):
        self.vehicle_local_position_ned_[idx]       =   numpy.array([msg.x,msg.y,msg.z],dtype=numpy.float64)
        self.vehicle_local_velocity_ned_[idx]       =   numpy.array([msg.vx,msg.vy,msg.vz],dtype=numpy.float64)

def main():
    # get the number of drones
    parser = argparse.ArgumentParser(description='Controller parameters')
    parser.add_argument('--num-drones','-n',type=int,default=numpy.uint8(1),help='input number of drones')
    arg_in = parser.parse_args()

    # initialize python rclpy library
    rclpy.init(args=None)

    print("Starting offboard control node...\n")
    offboard_control = offboard_mission_module(arg_in.num_drones)
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()