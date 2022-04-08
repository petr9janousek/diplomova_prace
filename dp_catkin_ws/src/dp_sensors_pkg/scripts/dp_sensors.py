#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from tf.transformations import euler_from_quaternion

class DPSensors():
    
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber("/leg_tracker/PeopleArray", PoseStamped, self.clbk_legs)
        self.pose_subscriber = rospy.Subscriber("/pozyx/pose", PoseStamped, self.clbk_pozyx)
        self.pose_subscriber = rospy.Subscriber("/pozyx/imu", PoseStamped, self.clbk_)

        self.pose_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.current_pose = Pose2D()
        self.target_pose = Pose2D()

        self.exit_flag = False
        rospy.on_shutdown(self.shutdownhook)

    def clbk_odom(self, odom_msg):
        self.current_pose = self.pose_flatten(odom_msg.pose)

    def check_collision(self):
        pass

    def pose_flatten(self, pose_3d):
        """pose_flatten - reduces pose from 3D to 2D using yaw as theta; 
        ROS devs recommendation is to transfer 3D and convert later, to avoid conversion loss during conversion for as long as possible

        Arguments:
            pose_3d -- pose with position and quaternion

        Returns:
            pose with x, y, theta
        """
        pose_2d = Pose2D()
        pose_2d.x = pose_3d.pose.position.x
        pose_2d.y = pose_3d.pose.position.y
        euler = euler_from_quaternion((pose_3d.pose.orientation.x,
                                                pose_3d.pose.orientation.y,
                                                pose_3d.pose.orientation.z,
                                                pose_3d.pose.orientation.w))
        pose_2d.theta = euler[2] #0 roll, 1 pitch, 2 yaw
        return pose_2d
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.exit_flag=True
            
if __name__ == '__main__':
    rospy.init_node('constroller_shortest')
      
    controller = DPSensors()

    try:
        controller.move()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()