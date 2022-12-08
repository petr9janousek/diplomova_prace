import rospy

class DP_PID():
    def __init__(self):
        self.setpoint = 0
        self.state = 0
        self.error = 0
        self.action = 0
        self.exit = False

        self._setpoint_pub() = 

        
    def shutdownhook():
        rospy.loginfo("shutdown time!")
        self.exit = True