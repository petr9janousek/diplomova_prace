#!/usr/bin/env python3

import signal

import rospy

from xbox360controller import Xbox360Controller as Xbox360

from geometry_msgs.msg import Twist
from dp_diffdrive_pkg.msg import MotorState

class Controller():
    twist_stop=Twist()
    twist_stop.linear.x=0
    twist_stop.angular.z=0

    def __init__(self):
        self._controller = Xbox360(0, axis_threshold=0.2)
        self._twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    
    def check_enable_switch(self):
        if not self._controller.button_a.is_pressed:
            self._twist_publisher.publish(Controller.twist_stop)
            self._controller.set_led(Xbox360.LED_OFF)
            #print("Move disabled")

    def on_button_pressed(self, button):
        #print('Tlačítko "{0} is {1}"'.format(button.name, button.is_pressed))
        if button.name == "button_a":
            self._controller.set_led(Xbox360.LED_ROTATE)
        elif button.name == "button_b":
            self._controller.set_led(Xbox360.LED_TOP_RIGHT_ON)
        elif button.name == "button_x":
            self._controller.set_led(Xbox360.LED_BOTTOM_LEFT_ON)
        elif button.name == "button_y":
            self._controller.set_led(Xbox360.LED_BOTTOM_RIGHT_ON)
        else:
            self._controller.set_rumble(0.5, 0.5, 500)

    def on_axis_moved(self, axis):
        if axis.name == "axis_r" or axis.name == "axis_l":
            #print('Osa "{0}"; souřadnice X={1} Y={2}'.format(axis.name, axis.x, axis.y))
            twist = Twist()
            
            twist.linear.x = -axis.y*20
            twist.angular.z = -axis.x*20
            
            self._twist_publisher.publish(twist)
        else:
            print('Osa "{0}"; souřadnice Z={1}'.format(axis.name, axis.value))
        
    def subscribeHandlers(self):
        # Button events
        self._controller.button_a.when_pressed = self.on_button_pressed
        self._controller.button_b.when_pressed = self.on_button_pressed
        self._controller.button_x.when_pressed = self.on_button_pressed
        self._controller.button_y.when_pressed = self.on_button_pressed
        self._controller.button_trigger_r.when_pressed = self.on_button_pressed
        self._controller.button_trigger_l.when_pressed = self.on_button_pressed

        # Axis move event
        self._controller.trigger_r.when_moved = self.on_axis_moved
        self._controller.trigger_l.when_moved = self.on_axis_moved
        self._controller.axis_l.when_moved = self.on_axis_moved
        self._controller.axis_r.when_moved = self.on_axis_moved


if __name__ == "__main__":
    rospy.init_node('teleop_twist_joystrick', log_level=rospy.INFO)
    stop_publisher = rospy.Publisher('/motor_stop', MotorState, queue_size = 1)
    
    #dead_man_switch = rospy.get_param("~dead_man", 1.0)

    #sets async read events
    try:
        x = Controller()
        x._controller.device_is_alive()            #check connections
    except RuntimeError as re:
        print(f"Exception occured during init, reason: {re}")
        exit(0)

    x.subscribeHandlers()   #if init succeeded connect handlers

    #loop deadman or timer
    rate = rospy.Rate(2) #2Hz
    while not rospy.is_shutdown():
        try:
            alive = x._controller.device_is_alive()
            if not alive:
                stop_publisher.publish(MotorState(True, False))
            rospy.sleep(0.01)
        except RuntimeError:
            stop_publisher.publish(MotorState(True, False))
            print("Device disconnected. Stopping movement...")

        x.check_enable_switch()
        #print("run")
        rate.sleep() #loop with rate 500ms
    
    #exit files cleanups
#####################################################################################
"""

            #rospy.spin() #waits for events
        except KeyboardInterrupt:
            #self.ctrl.close()
            pass
if __name__ == "__main__":
    fd = DPFaceDetector("cpu", True, "face")
    rospy.sleep(1)
    try:
        rate = rospy.Rate(4)
        cont = True
        while not rospy.is_shutdown():
            rate.sleep()
            if fd.image is not None and fd.depth is not None and cont:
                fd.face_detect()

            key = cv2.waitKey(30) 
            if key == ord('s'):
                cont = False
            elif key == ord('g'):
                cont = True
    except rospy.ROSInterruptException:
        print("exit")
"""
#####################################################################################
"""
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.exit_flag=True

if __name__ == '__main__':
    rospy.init_node('controller_pursuit', log_level=rospy.INFO)
      
    controller = DPControlPursuit()

    try:
        controller.move()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

"""
