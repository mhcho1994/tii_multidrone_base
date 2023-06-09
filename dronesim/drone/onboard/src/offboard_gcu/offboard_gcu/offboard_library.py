#!/usr/bin/python3

# python implementation of px4 multi-vehicle offboard control
# load public libraries
import numpy
import math

# load ROS2 related libraries

# load ROS2 messages

class flight_management():

    # define controller module parameters
    def load_parameters(self):

        # set the number of drones
        self.num_drones                             =   numpy.uint8(1)

        # initialize parameters for flight management module
        self.nav_xy_accept_rad_                     =   numpy.float32(2.0)
        self.nav_z_accept_rad_                      =   numpy.float32(0.8)
        self.nav_vz_accept_bound_                   =   numpy.float32(1.5)

        # initialize fields for vehicle command topic
        self.PX4_CUSTOM_MAIN_MODE_MANUAL_           =   numpy.uint8(1)
        self.PX4_CUSTOM_MAIN_MODE_ALTCTL_           =   numpy.uint8(2)
        self.PX4_CUSTOM_MAIN_MODE_POSCTL_           =   numpy.uint8(3)
        self.PX4_CUSTOM_MAIN_MODE_AUTO_             =   numpy.uint8(4)
        self.PX4_CUSTOM_MAIN_MODE_ACRO_             =   numpy.uint8(5)
        self.PX4_CUSTOM_MAIN_MODE_OFFBOARD_         =   numpy.uint8(6)
        self.PX4_CUSTOM_MAIN_MODE_STABILIZED_       =   numpy.uint8(7)

        self.PX4_CUSTOM_SUB_MODE_AUTO_READY_        =   numpy.uint8(1) 						        # debug: warning when engaged (unsupported auto mode)
        self.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF_      =   numpy.uint8(2)
        self.PX4_CUSTOM_SUB_MODE_AUTO_LOITER_       =   numpy.uint8(3)
        self.PX4_CUSTOM_SUB_MODE_AUTO_MISSION_      =   numpy.uint8(4)
        self.PX4_CUSTOM_SUB_MODE_AUTO_RTL_          =   numpy.uint8(5)
        self.PX4_CUSTOM_SUB_MODE_AUTO_LAND_         =   numpy.uint8(6)
        self.PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET_=   numpy.uint8(8)
        self.PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND_     =   numpy.uint8(9)
        self.PX4_CUSTOM_SUB_MODE_AUTO_VTOL_TAKEOFF_ =   numpy.uint8(10)

        self.ARMING_ACTION_DISARM_ 					=	numpy.uint8(0)
        self.ARMING_ACTION_ARM_ 					=	numpy.uint8(1)

        # initialize fields of vehicle status ARMING_STATE_ARMED_
        self.NAVIGATION_STATE_MANUAL_ 				= 	numpy.uint8(0)
        self.NAVIGATION_STATE_ALTCTL_ 				= 	numpy.uint8(1)
        self.NAVIGATION_STATE_POSCTL_ 				= 	numpy.uint8(2)
        self.NAVIGATION_STATE_AUTO_MISSION_ 		= 	numpy.uint8(3)
        self.NAVIGATION_STATE_AUTO_LOITER_ 			= 	numpy.uint8(4)
        self.NAVIGATION_STATE_AUTO_RTL_ 			= 	numpy.uint8(5)
        self.NAVIGATION_STATE_AUTO_LANDENGFAIL_ 	= 	numpy.uint8(8)
        self.NAVIGATION_STATE_ACRO_ 				=	numpy.uint8(10)
        self.NAVIGATION_STATE_DESCEND_ 				= 	numpy.uint8(12)
        self.NAVIGATION_STATE_TERMINATION_ 			= 	numpy.uint8(13)
        self.NAVIGATION_STATE_OFFBOARD_ 			= 	numpy.uint8(14)
        self.NAVIGATION_STATE_STAB_ 				= 	numpy.uint8(15)
        self.NAVIGATION_STATE_AUTO_TAKEOFF_ 		= 	numpy.uint8(17)
        self.NAVIGATION_STATE_AUTO_LAND_ 			= 	numpy.uint8(18)
        self.NAVIGATION_STATE_AUTO_FOLLOW_TARGET_ 	= 	numpy.uint8(19)
        self.NAVIGATION_STATE_AUTO_PRECLAND_ 		= 	numpy.uint8(20)
        self.NAVIGATION_STATE_ORBIT_ 				= 	numpy.uint8(21)
        self.NAVIGATION_STATE_AUTO_VTOL_TAKEOFF_ 	= 	numpy.uint8(22)

        self.ARMING_STATE_INIT_                     =   numpy.uint8(0)
        self.ARMING_STATE_STANDBY_                  =   numpy.uint8(1)
        self.ARMING_STATE_ARMED_                    =   numpy.uint8(2)

        # initialize fields of takeoff status topic
        self.TAKEOFF_STATE_UNINITIALIZED_     		= 	numpy.uint8(0)
        self.TAKEOFF_STATE_DISARMED_          		= 	numpy.uint8(1)
        self.TAKEOFF_STATE_SPOOLUP_           		= 	numpy.uint8(2)
        self.TAKEOFF_STATE_READY_FOR_TAKEOFF_       =   numpy.uint8(3)
        self.TAKEOFF_STATE_RAMPUP_                  =   numpy.uint8(4)
        self.TAKEOFF_STATE_FLIGHT_                  =   numpy.uint8(5)

        # waypoint reach condition radius
        self.nav_wpt_reach_rad_                     =   numpy.float32(0.25)

        # observer gain
        self.Lobv                                   =   numpy.array([[0.62,0.00,0.00],
                                                                     [0.00,0.62,0.00],
                                                                     [0.00,0.00,0.62]])

        # detector threshold
        self.detect_threshold                       =   numpy.float(1.25)

def get_projection_matrix(vector):

    proj_mat    =   numpy.matmul(numpy.atleast_2d(vector).T,numpy.atleast_2d(vector))
    proj_mat    =   proj_mat/numpy.dot(vector,vector)

    return proj_mat