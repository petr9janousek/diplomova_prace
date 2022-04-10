#!/usr/bin/env python3

import rospy
import freenect
from random import randint

class DPKinect():
    def __init__(self):
        #self.pose_subscriber = rospy.Subscriber("/odom", Odometry, self.clbk_odom)
        #self.pose_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.ctx = freenect.init()
        self.dev = freenect.open_device(self.ctx, 0)
        
        self.exit_flag = False
        rospy.on_shutdown(self.shutdownhook)
    
    def set_motor(self, val):
        """Set motor angle 
        0    - horizontal 
        1-29 - proportional
        30   - max tilt upwards
        """
        self.clip(val, 0, 30)
        freenect.set_tilt_degs(self.dev, val)

    def set_motor(self, val):
        """Set led color
        0 - off
        1 - green
        2 - red
        3 - yellow
        4 - green blink
        5 - green blink
        6 - red/yellow blink
        """
        self.clip(val, 0, 30)
        freenect.set_tilt_degs(self.dev, val)

    def motor_test(self, manual=False):
        rospy.loginfo("motor_test")
        if manual:
            tilt = int(input("INPUT motor tilt (0-30; ^D or ^\ for exit): "))
        else:
            tilt = randint(0, 30)
        freenect.set_tilt_degs(self.dev, tilt)

    def led_test(self, manual=False):
        rospy.loginfo("led_test")
        if manual:
            led = int(input("INPUT led state (0-6; ^D or ^\ for exit): "))
        else:
            led = randint(0, 6)
        freenect.set_led(self.dev, led)

    def clip(self, n, lowlim, highlim):
        """Crop number
        
        Arguments:
            lowlim: lower limit
            highlim: upper limit
            n: value to crop in place
        """
        return int(max(lowlim, min(n, highlim))) #int because float speed cant be set

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.exit_flag=True
            
if __name__ == '__main__':
    rospy.init_node('measure_node')
    k = DPKinect()
    try:
        while True:
            k.motor_test(manual=True)
            k.led_test(manual=True)
    except rospy.ROSInterruptException:
        k.shutdownhook()

    #rospy.spin() end peacefully...