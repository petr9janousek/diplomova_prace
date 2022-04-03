#!/usr/bin/env python

import rospy, math, enum
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Mode(enum.IntEnum):
    ROTATE = 1
    TRANSLATE = 2
    ORIENTATE = 3
    DONE = 4

class DPControlShortest():
    
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.clbk_goal)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.clbk_odom)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.current_pose = Pose2D()
        self.target_pose = Pose2D()

        self.YAW_TOLERANCE = math.radians(4)
        self.DIST_TOLERANCE = 0.5

        self.exit_flag = False
        rospy.on_shutdown(self.shutdownhook)

    def clbk_goal(self, pose_msg):
        self.target_pose = self.pose_flatten(pose_msg)
        rospy.logwarn(f"Target pose:\n{self.target_pose}")
        self.mode = Mode.ROTATE #position has changed restore mode to rotation

    def clbk_odom(self, odom_msg):
        self.current_pose = self.pose_flatten(odom_msg.pose)
        #rospy.loginfo_throttle(1,f"Current pose:\n{self.current_pose}")

    def rotation_check(self):
        dx = self.target_pose.x - self.current_pose.x
        dy = self.target_pose.y - self.current_pose.y

        yaw_to_target = math.atan2(dy,dx)
        yaw_error = yaw_to_target - self.current_pose.theta # e = w - y
        
        if math.fabs(yaw_error) > self.YAW_TOLERANCE: 
            self.mode = Mode.ROTATE

    def rotation(self):
        dx = self.target_pose.x - self.current_pose.x
        dy = self.target_pose.y - self.current_pose.y

        yaw_to_target = math.atan2(dy,dx)
        yaw_error = yaw_to_target - self.current_pose.theta # e = w - y

        if yaw_error > math.pi:
            yaw_error = -(2*math.pi)+yaw_error
        elif yaw_error < -math.pi:
            yaw_error = (2*math.pi)+yaw_error

        rospy.loginfo(f"Yaw error: {yaw_error} rad; {math.degrees(yaw_error)} deg")
        yaw_msg = Twist()
        
        if math.fabs(yaw_error) > self.YAW_TOLERANCE: 
            yaw_msg.angular.z = 0.6 if yaw_error > 0 else -0.6 #yaw -pi:pi
            self.twist_publisher.publish(yaw_msg)
            #self.mode = Mode.ROTATE
        else:
            yaw_msg.angular.z = 0
            self.twist_publisher.publish(yaw_msg)
            rospy.logwarn("Changing state to: TRANSLATE")
            self.mode = Mode.TRANSLATE

            rospy.sleep(0.2)
        

    def translation(self):   
        self.rotation_check()

        #x**2 + y**2
        dist_dx = self.target_pose.x - self.current_pose.x
        dist_dy = self.target_pose.y - self.current_pose.y
        dist_error = math.sqrt(dist_dx*dist_dx + dist_dy*dist_dy)

        rospy.loginfo(f"Dist error: {dist_error} m")

        dist_msg = Twist()
        if dist_error > self.DIST_TOLERANCE:
            dist_msg.linear.x = 1
            self.twist_publisher.publish(dist_msg)
        else:
            rospy.logwarn("Changing state: to ORIENTATE")
            self.mode = Mode.ORIENTATE
            dist_msg.linear.x = 0
            self.twist_publisher.publish(dist_msg)


    def orientation(self):
        yaw_error = self.target_pose.theta - self.current_pose.theta
        
        rospy.loginfo(f"Yaw error: {yaw_error} deg;\t{math.degrees(yaw_error)} rad;")
        
        if yaw_error > math.pi:
            yaw_error = -(2*math.pi)+yaw_error
        elif yaw_error < -math.pi:
            yaw_error = (2*math.pi)+yaw_error
            
        yaw_msg = Twist()
        
        if math.fabs(yaw_error) > self.YAW_TOLERANCE: 
            yaw_msg.angular.z = 0.6 if yaw_error > 0 else -0.6 #yaw -pi:pi
            self.twist_publisher.publish(yaw_msg)
        else:
            yaw_msg.angular.z = 0
            self.twist_publisher.publish(yaw_msg)
            self.mode = Mode.DONE

            
    def move(self):
        self.mode = Mode.DONE
        call_freq = rospy.Rate(20)
        while not self.exit_flag:
            if self.mode  == Mode.ROTATE:
                self.rotation()

            elif self.mode  == Mode.TRANSLATE:
                self.translation()

            elif self.mode  == Mode.ORIENTATE:
                self.orientation()

            elif self.mode  == Mode.DONE:
                stop_msg = Twist()
                stop_msg.linear.x = 0
                stop_msg.angular.z = 0
                self.twist_publisher.publish(stop_msg)

            call_freq.sleep()

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
      
    controller = DPControlShortest()

    try:
        controller.move()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()