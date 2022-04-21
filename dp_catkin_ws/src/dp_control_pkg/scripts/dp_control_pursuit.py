#!/usr/bin/env python
import rospy, math, enum
from geometry_msgs.msg import Twist, PoseStamped, Pose2D, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Mode(enum.IntEnum):
    ROTATE = 1
    TRANSLATE = 2
    ORIENTATE = 3
    DONE = 4

class DPControlPursuit():
    def __init__(self):
        """__init__ Create Pure Pursuit controller, with TRANSLATE-ROTATE steps kinematics
        """
        
        #self.pose_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.clbk_goal)  # USE RVIZ TO OBTAIN POSE
        self.pose_subscriber = rospy.Subscriber("/decoy_pose", Pose, self.clbk_goal)            # USE DECOY TO OBTAIN POSE
        self.pose_subscriber = rospy.Subscriber("/detect_leg_clusters", Pose, self.clbk_goal)            # USE DECOY TO OBTAIN POSE
        #self.pose_subscriber = rospy.Subscriber("/pozyx_pose", PoseStamped, self.clbk_goal)            # USE POZYX TO OBTAIN POSE
        
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.clbk_odom)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.current_pose = Pose2D()
        self.target_pose = Pose2D()

        self.YAW_TOLERANCE = math.radians(1) #faster later, no transforms needed, 1 degree
        self.DIST_TOLERANCE = 1.5 # meters

        self.exit_flag = False
        rospy.on_shutdown(self.shutdownhook)

    def clbk_goal(self, pose_msg):
        self.target_pose = self.pose_flatten(pose_msg)
        rospy.logdebug(f"Target pose:\n{self.target_pose}")

    def clbk_odom(self, odom_msg):
        self.current_pose = self.pose_flatten(odom_msg.pose)
        rospy.logdebug(f"Current pose:\n{self.current_pose}")

    def find_arc(self, msg):
        #x**2 + y**2
        dist_dx = self.target_pose.x - self.current_pose.x
        dist_dy = self.target_pose.y - self.current_pose.y
        dist_error = math.sqrt(dist_dx*dist_dx + dist_dy*dist_dy)

        rospy.loginfo(f"Dist error: {dist_error} m")

        if dist_error > self.DIST_TOLERANCE:
            msg.linear.x = 1.1
            msg.angular.z = dist_dx*(-2/dist_error**2)
        elif dist_error < self.DIST_TOLERANCE/2:
            msg.linear.x = -0.5
            msg.angular.z = 0
        else:
            msg.linear.x = 0
            msg.angular.z = 0

    def move(self):
        """move Waits for target to periodicaly change and than follow
        """
        self.mode = Mode.DONE
        call_freq = rospy.Rate(10)
        while not self.exit_flag:
            msg = Twist()
            
            self.find_arc(msg)

            self.twist_publisher.publish(msg)

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

        return pose_2d
        
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