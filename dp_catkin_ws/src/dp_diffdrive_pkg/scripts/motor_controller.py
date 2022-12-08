#!/usr/bin/env python3

import rospy
import roslib
from ZLAC8015D import *
from std_msgs.msg import Int32, Float32, Bool
from dp_diffdrive_pkg.msg import MotorState
import time, enum
from geometry_msgs.msg import Twist 
import math

class MotorInterface:
    def __init__(self):     
        rospy.init_node("motor_interface_node")
        rospy.loginfo(f"{rospy.get_name()} started")
        ### parameters ###
        # CHANGE WILL BREAK THE PD REGULATOR - NEW IDENT MIGHT BE NEEDED
        self.accel_time = rospy.get_param('~accel_time', 200.0)  # S curve acceleratión time defaults to 200 
        self.decel_time = rospy.get_param('~decel_time', 200.0)  # S curve deceleratión time defaults to 200
        #"ticks_meter"7669.39584
        self.hw_l = rospy.get_param('~hw_l', 0.755)
        self.hw_d = rospy.get_param('~hw_d', 0.170)
        self.rate = float(rospy.get_param('~rate', 20.0))
        # CHANGE WILL BREAK THE PD REGULATOR - NEW IDENT MIGHT BE NEEDED
        self.limit_speed = rospy.get_param('speed_limit', 80)  

        ### motor prep ###
        self.motors = ZLAC8015D()
        self.motors.disable_motor()

        self.motors.set_accel_time(self.accel_time, self.accel_time)
        self.motors.set_decel_time(self.decel_time, self.decel_time)

        self.motors.clear_feedback()
        self.motors.set_mode(3) #speed control mode
       
        self.motors.external_stop()
        self.ESTOP = False
        """
        self.deact_check = False
        self.deact_run = False
        self.deact_time = rospy.Time.now()
        """
        
        ### publishers ###
        #self.pub_odom = rospy.Publisher('/lwheel', Int32, queue_size=1) #for diff_tf
        #self.broadcaster_odom = TransformBroadcaster()
        self.pub_lwheel = rospy.Publisher('/lwheel', Int32, queue_size=1) #for diff_tf
        self.pub_rwheel = rospy.Publisher('/rwheel', Int32, queue_size=1)

        ### subscribers ###
        self.sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.clbk_twist)
        self.sub_rwheel = rospy.Subscriber("/motor_stop", MotorState, self.clbk_motor_stop)

        ### shutdown ###
        rospy.on_shutdown(self.shutdownhook)
        self.shutdown_flag = False

        ### enable motors ### 
        self.motors.enable_motor()
        
    
    def clbk_motor_stop(self, motor_state):
        if motor_state.emergency == True:
            rospy.logwarn("Emergency stop activated")
            self.motors.emergency_stop()
            self.ESTOP = True
        elif self.ESTOP is True:
            rospy.logwarn("Emergency stop deactivated")
            self.motors.set_rpm(0,0)
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

    def obtain_info(self):
        rate = rospy.Rate(int(self.rate))               #encoder published @20Hz
        while not self.shutdown_flag:
            rate.sleep()
            l_count, r_count = self.motors.get_pulse_count()
            rospy.loginfo_throttle(2,f"encoder l: {l_count}, r: {r_count}")

            vl, vr = self.motors.get_linear_velocities()
            rospy.loginfo_throttle(2,f"speed l: {vl}, r: {vr}")

            rpml, rpmr = self.motors.get_rpm()
            rospy.loginfo_throttle(2,f"rpm l: {rpml}, r: {rpml}")

            tl, tr = self.motors.get_wheels_travelled()
            rospy.loginfo_throttle(2,f"travelled l: {tr}, r: {tl}")
            rospy.loginfo_throttle(2,f"---------------------------")

    def publish_encoders(self):
        rate = rospy.Rate(self.rate)               #encoder published @20Hz
        while not self.shutdown_flag:
            rate.sleep()
            l_count, r_count = self.motors.get_pulse_count()
            #self.motors.get_linear_velocities()
            self.pub_lwheel.publish(l_count)
            self.pub_rwheel.publish(r_count)
            #rospy.logdebug(f"lw_enc: {l_count}, rw_enc: {r_count}")
            #rospy.loginfo_throttle(2, f"rpm: {self.motors.get_rpm()}")
            #self.timer_deactivate()

    def clbk_twist(self, twist):
        """Callback for twist message. Translates twist to motor commands. 
        Limits speed to set maximum, default 20."""
        # if self.deact_check:
        #     rospy.logwarn("deact_check")
        #     self.deact_check = False
        #     self.motors.enable_motor()
        
        v = twist.linear.x  #in meters per second
        w = twist.angular.z #in radians per second

        v_r = (2*v+w*self.hw_l)/2 #2*r = d = 0.170
        v_l = (2*v-w*self.hw_l)/2 #in radians per second
        
        #2xRPS = 1m/s (2*pi*0,170=1m)   } *2
        #RPS = 60*RPM                   } *2*60 =*120
        #m = 100 cm => /100             } *120/100 = *1.2
        #v_r = v_r * 1.2
        #v_l = v_l * 1.2
        
        v_r = self.crop_speed(v_r, -self.limit_speed, self.limit_speed)
        v_l = self.crop_speed(v_l, -self.limit_speed, self.limit_speed)

        rospy.loginfo(f"controller::pub vr:{v_l} vl:{v_r}")
        #vr, vl = self.crop_speed(v_r, v_l, 50,50)
        self.motors.set_rpm(int(v_l),int(v_r))

    def crop_speed(self, n, lowlim, highlim):
        """Crop speed limit, simple crop function
        
        Arguments:
            lowlim: lower limit
            highlim: upper limit
            n: value to crop in place
        """
        return int(max(lowlim, min(n, highlim))) #int because float speed cant be set

    """
    def timer_deactivate(self):
        rospy.loginfo_throttle(1,"deact")
        l,r = self.motors.get_rpm()
        if l == 0.0 and r == 0.0 and self.deact_run == False:

            rospy.loginfo("activated")
            self.deact_run = True
            self.deact_time = rospy.Time.now()

        if self.deact_run:
            rospy.loginfo(f"time {rospy.Time.now() - self.deact_time}")
            if rospy.Time.now() - self.deact_time > rospy.Duration(10):
                self.deact_run = False
                self.deact_check = True
                self.motors.disable_motor()
    """

    def shutdownhook(self):
        """Callback for ROS shutting down the node. If loop is running shutdown_flag notfication might be used."""
        self.motors.disable_motor()
        self.shutdown_flag = True
        
if __name__ == "__main__":
    mi = MotorInterface()
    try:
        mi.publish_encoders()
    except rospy.ROSInterruptException:
        mi.shutdownhook()
