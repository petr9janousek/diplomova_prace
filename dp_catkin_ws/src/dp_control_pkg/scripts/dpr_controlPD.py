#!/usr/bin/env python3

import rospy, math, enum
from geometry_msgs.msg import Twist, Pose, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from numpy import interp

class Mode(enum.IntEnum):
    ROTATE = 1
    TRANSLATE = 2
    ORIENTATE = 3
    DONE = 4

class ControllerPD():
    YAW_TOLERANCE = math.radians(25)
    DIST_TOLERANCE = 0.5
    DIST_MAX = 15

    def __init__(self):
        #self.pose_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.clbk_goal)
        self.pose_subscriber = rospy.Subscriber("/uwb/position/pose", Pose, self.clbk_goal)  
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.angle = 0
        self.distance = 0
        
        self.run = False
        self.exit_flag = False
        rospy.on_shutdown(self.shutdownhook)

    def clbk_goal(self, pose_msg):
        target = self.pose_flatten(pose_msg)
        #rospy.loginfo_throttle(0.5,f"\nTarget pose:\n{target}")
        
        self.angle = math.atan2(target.y,target.x)
        self.distance = math.sqrt(target.x**2 + target.y**2)
        msg = Twist()     

        if self.distance <= self.DIST_TOLERANCE or self.distance >= self.DIST_MAX:
            self.twist_publisher.publish(msg)
            return

        if self.angle > math.pi:
            self.angle = -(2*math.pi)+self.angle
        elif self.angle < -math.pi:
            self.angle = (2*math.pi)+self.angle
        
        #rospy.logwarn_throttle(0.5,f"\nYaw: yaw to target {math.degrees(self.angle)}")
        
        if self.angle > self.YAW_TOLERANCE:
            tmp= interp(self.angle, [0,math.pi],[40,160])
            rospy.loginfo(f"angle:{self.angle}  interp:{tmp}")
            msg.angular.z = tmp
            
        elif self.angle < -self.YAW_TOLERANCE:
            tmp = interp(self.angle, [-math.pi,0],[-140,-40])
            rospy.loginfo(f"angle:{self.angle}  interp:{tmp}")
            msg.angular.z = tmp
        else:
            msg.angular.z = 0

        if self.distance > self.DIST_TOLERANCE:
            tmp = interp(self.distance, [1, 2], [20,70])
            rospy.loginfo(f"dist:{self.distance}  interp:{tmp}")
            msg.linear.x = tmp

        self.twist_publisher.publish(msg)

    def move(self):
        pass 
        """i = 0
        while not self.exit_flag:
            i = i + 1
            rospy.loginfo(i)
        """

    def pose_flatten(self, pose_3d):
        """pose_flatten - reduces pose from 3D to 2D using yaw as theta; 
        ROS devs recommendation is to transfer 3D and convert later, to avoid conversion loss during conversion for as long as possible

        Arguments:
            pose_3d -- pose with position and quaternion

        Returns:
            pose with x, y, theta
        """
        pose_2d = Pose2D()
        
        if type(pose_3d) is Pose:
            pose_2d.x = pose_3d.position.x
            pose_2d.y = pose_3d.position.y 
            euler = euler_from_quaternion((pose_3d.orientation.x,           
                                                    pose_3d.orientation.y,  
                                                    pose_3d.orientation.z,  
                                                    pose_3d.orientation.w))
            pose_2d.theta = euler[2] #0 roll, 1 pitch, 2 yaw
        else:
            pose_2d.x = pose_3d.pose.position.x
            pose_2d.y = pose_3d.pose.position.y 
            euler = euler_from_quaternion((pose_3d.pose.orientation.x,          
                                                    pose_3d.pose.orientation.y, 
                                                    pose_3d.pose.orientation.z, 
                                                    pose_3d.pose.orientation.w))
            pose_2d.theta = euler[2] #0 roll, 1 pitch, 2 yaw

        #rospy.loginfo(pose_2d)
        return pose_2d

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.exit_flag=True
            
if __name__ == '__main__':
    rospy.init_node('constroller_shortest')
      
    controller = ControllerPD()

    try:
        controller.move()
    except rospy.ROSInterruptException:
        controller.shutdownhook = True

    rospy.spin()