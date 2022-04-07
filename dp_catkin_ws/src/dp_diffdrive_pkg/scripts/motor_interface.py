#!/usr/bin/env python3

import rospy
import roslib
from ZLAC8015D import *
from std_msgs.msg import Int32, Float32, Bool
from dp_diffdrive_pkg.msg import MotorState
import time, enum

class MotorInterface:
    def __init__(self):     
        rospy.init_node("motor_interface_node")
        rospy.loginfo(f"{rospy.get_name()} started")

        ### parameters ###
        self.accel_time = rospy.get_param('~accel_time',200.0)  # S curve acceleratión time defaults to 1000
        self.decel_time = rospy.get_param('~decel_time',200.0)  # S curve deceleratión time defaults to 1000

        ### motor prep ###
        self.motors = ZLAC8015D()
        self.motors.disable_motor()

        self.motors.set_accel_time(self.accel_time, self.accel_time)
        self.motors.set_decel_time(self.decel_time, self.decel_time)

        self.motors.set_mode(3) #speed control mode

        ### publishers ###
        self.pub_lwheel = rospy.Publisher('/lwheel', Int32, queue_size=1) #for diff_tf
        self.pub_rwheel = rospy.Publisher('/rwheel', Int32, queue_size=1)

        ### subscribers ###
        self.sub_lwheel = rospy.Subscriber('/lwheel_vtarget', Float32, self.clbk_motor_left) #for twist_to_motors
        self.sub_rwheel = rospy.Subscriber("/rwheel_vtarget", Float32, self.clbk_motor_right)

        self.sub_rwheel = rospy.Subscriber("/motor_stop", MotorState, self.clbk_motor_stop)

        ### shutdown ###
        rospy.on_shutdown(self.shutdownhook)
        self.shutdown_flag = False

        self.motors.enable_motor()
        self.ESTOP = False
    
    def clbk_motor_stop(self, motor_state):
        if motor_state.emergency == True:
            rospy.logwarn("Emergency stop activated")
            self.motors.emergency_stop()
            self.ESTOP = True
        elif self.ESTOP is True:
            rospy.logwarn("Emergency stop deactivated")
            self.motors.enable_motor()
            self.ESTOP = False
        elif motor_state.enabled == True:
                rospy.loginfo("Motors activated")
                self.motors.enable_motor()
        elif motor_state.enabled == False:
                rospy.loginfo("Motors deactivated")
                self.motors.disable_motor()
        else:
            rospy.logwarn(f"Unknown state transition, state: {motor_state}, estop: {self.ESTOP}")
        

    def clbk_motor_left(self, target_rpm):    #in m/s
        self.motors.set_rpm_single(int(target_rpm.data), isLeft=True)
        rospy.loginfo(f"left_vtarget: {target_rpm}")

    def clbk_motor_right(self, target_rpm):  # in m/s
        self.motors.set_rpm_single(int(target_rpm.data), isLeft=False)
        rospy.loginfo(f"right_vtarget: {target_rpm}")

    def publish_encoders(self):
        rate = rospy.Rate(20)               #encoder published @20Hz
        while not self.shutdown_flag:
            rate.sleep()
            l_count, r_count = self.motors.get_pulse_count()
            self.pub_lwheel.publish(l_count)
            self.pub_rwheel.publish(r_count)
            rospy.logdebug(f"lw_enc: {l_count}, rw_enc: {r_count}")
            rospy.loginfo_throttle(2, f"rpm: {self.motors.get_rpm()}")

    def shutdownhook(self):
        self.motors.disable_motor()
        self.shutdown_flag = True
        
if __name__ == "__main__":
    mi = MotorInterface()
    try:
        mi.publish_encoders()
    except rospy.ROSInterruptException:
        mi.shutdownhook()