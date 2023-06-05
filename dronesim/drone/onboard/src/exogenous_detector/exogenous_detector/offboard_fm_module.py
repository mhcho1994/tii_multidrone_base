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
from .offboard_library import flight_management

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
        self.vehicle_status_subscriber_             =   self.create_subscription(VehicleStatus,f'/{fmu[0]}/fmu/out/vehicle_status',self.process_vehicle_status,qos_profile_sub)
        self.takeoff_status_subscriber_             =   self.create_subscription(TakeoffStatus,f'/{fmu[0]}/fmu/out/takeoff_status',self.process_takeoff_status,qos_profile_sub)
        self.vehicle_local_position_subscriber_     =   self.create_subscription(VehicleLocalPosition,f'/{fmu[0]}/fmu/out/vehicle_local_position',self.process_vehicle_local_position,qos_profile_sub)
        # self.vehicle_odometry_subscriber_           =   self.create_subscription(VehicleOdometry,f'{fmu}/out/vehicle_odometry',self.process_vehicle_odometry,qos_profile_sub)

        # checking subscribers
        # self.start_vehicle_odometry_                =   wait_for_message(self,VehicleOdometry,f'{fmu}/out/vehicle_odometry')

        # initialize parameters for timer callback function
        self.timer_period                           =   numpy.float32(0.1)  # 100 milliseconds
        self.timer_                                 =   self.create_timer(self.timer_period,self.obfm_callback)
        self.temp_callback_counter_                 =   numpy.uint64(0)
        self.total_callback_counter_                =   numpy.uint64(0)
        self.timestamp_checker_                     =	numpy.uint64(0)

        # initialize parameters for flight management module
        self.flight_phase_ 	                        =	numpy.uint8(0)
        self.entry_execute_                         =	numpy.uint8(1)

        # initialize fields of vehicle status topic
        self.status_nav_state_                      =   numpy.uint8(0)
        self.status_arming_status_                  =   numpy.uint8(0)

        # initialize fields for trajectory setpoint topic
        self.trajectory_setpoint_x_ 				=	numpy.float32(0.0)
        self.trajectory_setpoint_y_ 				=	numpy.float32(0.0)
        self.trajectory_setpoint_z_ 				=	numpy.float32(0.0)
        self.trajectory_setpoint_yaw_ 				=	numpy.float32(0.0)

        # initialize fields of offboard control mode topic
        self.offboard_ctrl_position_                =	False
        self.offboard_ctrl_velocity_                =	False
        self.offboard_ctrl_acceleration_            =	False
        self.offboard_ctrl_attitude_                =	False
        self.offboard_ctrl_body_rate_	            =	False
        self.offboard_ctrl_actuator_                =	False

        # declare subscribers once to avoid unused variable warning
        self.vehicle_status_subscriber_
        self.takeoff_status_subscriber_
        self.vehicle_local_position_subscriber_
        # self.vehicle_odometry_subscriber_ 

    # offboard flight management callback function
    def obfm_callback(self):

        # ------------------- Offboard control test code -------------------
        for idx in range(self.num_drones):
            if (self.total_callback_counter_ <= numpy.uint64(10/self.timer_period)):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1, \
                    param2=self.PX4_CUSTOM_MAIN_MODE_MANUAL_,idx=idx+1)

            elif (self.total_callback_counter_ > numpy.uint64(10/self.timer_period)) \
                and (self.total_callback_counter_ <= numpy.uint64(15/self.timer_period)):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=self.ARMING_ACTION_ARM_,idx=idx+1)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1, \
                    param2=self.PX4_CUSTOM_MAIN_MODE_AUTO_,param3=self.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF_,idx=idx+1)

            elif (self.total_callback_counter_ > numpy.uint64(15/self.timer_period)):
                self.trajectory_setpoint_z_ 				=	numpy.float32(-20.0)
                self.publish_trajectory_setpoint(idx=idx+1)
                self.offboard_ctrl_position_  = True
                self.publish_offboard_control_mode(idx=idx+1)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_OFFBOARD_,idx=idx+1)

            else:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_OFFBOARD_,idx=idx+1)

        # ------------------- mission flight using offboard control -------------------
        # create a log message at NuttShell
        # self.get_logger().info('Offboard Mission Flight Manager Callback')

        # for testing the developed spoofing attack detector
        # need to be revised when the code is built and integrated in the real hardware

        # phase 0: manual(idle) - switch to manual mode (from any mode of last missions)
        # Set PX4_CUSTOM_MAIN_MODE to PX4_CUSTOM_MAIN_MODE_MANUAL and hold this mode for 10 seconds
        # if self.flight_phase_ == 0:
            
        #     # entry:
        #     if self.entry_execute_:

        #         self.entry_execute_ 			= 	numpy.uint64(0)
        #         self.temp_callback_counter_		=   numpy.uint64(0)
        #         self.total_lapse_counter_  		=   numpy.uint64(0)
            
        #     # during:
        #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1,param2=self.PX4_CUSTOM_MAIN_MODE_MANUAL_)
        #     self.get_logger().info("Current Mode: Manual (Idle)")

        #     # transition:
        #     if 0 #(self.status_nav_state_ == self.NAVIGATION_STATE_MANUAL_) and (self.temp_callback_counter_ > numpy.uint64(10/self.timer_period)):

        #         # exit:
        #         self.flight_phase_ 			=	1
        #         self.entry_execute_ 		=	1

        self.temp_callback_counter_             =   self.temp_callback_counter_+1
        self.total_callback_counter_            =   self.total_callback_counter_+1

    # publisher functions
    def publish_vehicle_command(self,command,param1=float(0.0),param2=float(0.0),param3=float(0.0),idx=numpy.uint8(1)):
        msg                                         =   VehicleCommand()
        msg.timestamp                               =   int(rclpy.clock.Clock().now().nanoseconds/1000) # time in microseconds
        msg.param1                                  =   float(param1)
        msg.param2                                  =   float(param2)
        msg.param3                                  =   float(param3)
        msg.command                                 =   command # command ID
        msg.target_system                           =   idx+1 # system which should execute the command
        msg.target_component                        =   1 # component which should execute the command, 0 for all components
        msg.source_system                           =   idx+1 # system sending the command
        msg.source_component                        =   1 # component sending the command
        msg.from_external                           =   True

        self.vehicle_command_publisher_[idx-1].publish(msg)

    def publish_trajectory_setpoint(self,idx=numpy.uint8(1)):
        msg                                         =   TrajectorySetpoint()
        msg.timestamp                               =   int(rclpy.clock.Clock().now().nanoseconds / 1000) # time in microseconds
        msg.position[0]                             =   float(self.trajectory_setpoint_x_)
        msg.position[1]                             =   float(self.trajectory_setpoint_y_)
        msg.position[2]                             =   float(self.trajectory_setpoint_z_)
        msg.yaw                                     =   float(self.trajectory_setpoint_yaw_)

        self.trajectory_setpoint_publisher_[idx-1].publish(msg)

    def publish_offboard_control_mode(self,idx=numpy.uint8(1)):
        msg                                         =   OffboardControlMode()
        msg.timestamp                               =   int(rclpy.clock.Clock().now().nanoseconds / 1000) # time in microseconds
        msg.position                                =   self.offboard_ctrl_position_ 
        msg.velocity                                =   self.offboard_ctrl_velocity_ 
        msg.acceleration                            =   self.offboard_ctrl_acceleration_
        msg.attitude                                =   self.offboard_ctrl_attitude_
        msg.body_rate                               =   self.offboard_ctrl_body_rate_

        self.offboard_control_mode_publisher_[idx-1].publish(msg)

    # Subscriber functions
    def process_vehicle_status(self,msg):
        self.status_nav_state_                      =   numpy.uint8(msg.nav_state)
        self.status_arming_status_                  =   numpy.uint8(msg.arming_state)

    def process_takeoff_status(self,msg):
        self.status_takeoff_                        =   numpy.uint8(msg.takeoff_state)

    def process_vehicle_local_position(self,msg):
        self.vehicle_local_position_x_              =   numpy.float32(msg.x)
        self.vehicle_local_position_y_              =   numpy.float32(msg.y)
        self.vehicle_local_position_z_              =   numpy.float32(msg.z)
        self.vehicle_local_position_vx_             =   numpy.float32(msg.vx)
        self.vehicle_local_position_vy_             =   numpy.float32(msg.vy)
        self.vehicle_local_position_vz_             =   numpy.float32(msg.vz)


# def wait_for_message(node:rclpy.node, msg_type, topic, time_out=10):
    
#     class _wfm():
#         def __init__(self) -> None:
#             self.time_out   =   time_out
#             self.msg        =   None
        
#         def callback(self, msg):
#             self.msg        =   msg

#     elapsed         =   0
#     wfm             =   _wfm()
#     subscription    =   node.create_subscription(msg_type, topic, wfm.callback, qos_profile=qos_profile_sub)

#     # rate = node.create_rate(10)
#     while rclpy.ok():
#         if wfm.msg != None: 
#             return wfm.msg

#         node.get_logger().info(f'waiting for {topic} ...')
#         rclpy.spin_once(node)
#         time.sleep(0.1)
#         elapsed += 0.1

#         if elapsed >= wfm.time_out:
#             node.get_logger().warn(f'time out waiting for {topic}...')
#             return None

#     subscription.destroy()

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