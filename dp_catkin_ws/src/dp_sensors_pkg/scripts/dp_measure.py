#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, Pose2D
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class DPMeasure():
    def __init__(self):
        #self.pose_subscriber = rospy.Subscriber("/odom", Odometry, self.clbk_odom)
        self.pose_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.exit_flag = False
        rospy.on_shutdown(self.shutdownhook)
        
        self.measure = False
        self.measure_freq = rospy.get_param('rate', 100)
        self.measure_dration = rospy.get_param('run_time', 1)
        self.measure_speed = rospy.get_param('speed', 20)

    def clbk_odom(self, odom_msg):
        """Too slow for accurate measurement! Use with combination with DIFF_TF, according to launch file measure.launch

        Arguments:
            odom_msg: Header + Pose_Stamped
        """
        if self.measure:
            o = self.pose_flatten(odom_msg.pose)
            t = rospy.Time.now().nsecs
            print(f"{t}, {o.theta}")

    def static_char(self):
        pass

    def dynamic_char(self):
        twist = Twist()
        twist.linear.x = 0; twist.angular.z = 0;
        self.pose_publisher.publish(twist)
        rospy.sleep(1)

        twist.angular.z = self.measure_speed
        self.pose_publisher.publish(twist)
        rospy.sleep(self.measure_dration)

        twist.angular.z = 0
        self.pose_publisher.publish(twist)

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
    rospy.init_node('measure_node')
      
    m = DPMeasure()

    try:
        m.dynamic_char()
    except rospy.ROSInterruptException:
        pass

    #rospy.spin()
    #end peacefully...