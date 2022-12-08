#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Pose, Pose2D, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from tf.transformations import euler_from_quaternion

from dpr_measure import PozyxMeasure

import time

class PozyxPublish():
    def __init__(self):
        rid = 0x6a5f                   #rospy.get_param('~remote_id', '0x6a2c')
        
        self.rate = rospy.Rate(20)      #float(rospy.get_param('~rate', '20.0'))
        #self.pub_pose = True            #rospy.get_param('~pub_pose', 'false')
        #self.pub_pose2d = True          #rospy.get_param('~pub_pose2d', 'true')
        
        self.pub_pose = rospy.Publisher('/uwb/position/pose', Pose, queue_size=1)
        self.pub_pose2d = rospy.Publisher('/uwb/position/pose2d', Pose2D, queue_size=1)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

        self.measure_device = PozyxMeasure(remote_id=rid)
        
        self.rec_err = 0 #can init to 0 as pos_count always is at least 1 in publish pos -> 1/1 -> no /0 exception
        self.rec_ok = 0 #can init to 0 as pos_count always is at least 1 -> 1/1 -> no /0 exception
        
        self.marker_id = 0
        
        self.time_prev = time.time()
        self.exit_flag = False
        rospy.on_shutdown(self.shutdown_hook)
    
    def loop_endless(self):
        while not self.exit_flag:
            status = self.pub_position3d()
            if status is None:
                continue
            """
            status = self.pub_position2d()
            if status is None:
                continue
            """
    def shutdown_hook(self):
        self.exit_flag=True

    def pub_marker(self, message: Pose or Pose2D, color: ColorRGBA, frame:str="link_base", lifetime:int=2.5) -> None:
        """set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3"""   
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()

        #just increment id as the value is unused anyway
        self.marker_id += 1 
        marker.id = self.marker_id 

        # Set the scale of the marker
        marker.scale.x, marker.scale.y, marker.scale.z = (0.1, 0.1, 0.1)
        # Set the color
        marker.color = color

        # Set the pose of the marker
        if type(message) is Pose:
            marker.pose = message
            marker.type = 1 #Cube (oriented)
        elif type(message) is Pose2D: 
            marker.pose = Pose(Point(message.x, message.y, PozyxMeasure.height/1000), Quaternion(0,0,0,1))
            marker.type = 2 #Sphere (no orientation)
        else: #must be pose2d as type is hinted in definition
            rospy.logerr(f"Tried to publish marker for unsupported type {type(message)}")
            return #not a critical error, used for debug only, log should be enough 

        marker.lifetime = rospy.Duration(lifetime)

        rospy.logdebug(f"Marker published in: {frame}\n{message}")
        self.marker_pub.publish(marker)

    def pub_position3d(self):
        p = self.measure_device.get_position()
        q = self.measure_device.get_quaterion()
        
        if p is None or q is None:
            self.rec_err += 1
            return # if data is corrupt dont publish
        
        self.rec_ok += 1
        success_average = (self.rec_ok/(self.rec_ok+self.rec_err))*100

        rospy.logdebug(f"Pozyx read success rate: {success_average:.2f} %")        
        pose3d = Pose(Point(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w))

        self.pub_pose.publish(pose3d)
        rospy.loginfo_throttle(0.5, f"Publish 3D: \n{pose3d}")
        self.pub_marker(pose3d,ColorRGBA(1,0,0,1))
        return True

    def pub_position2d(self):
        p = self.measure_device.get_position()
        a = self.measure_device.get_eulerAngles()

        if p is None or a is None:
            self.rec_err += 1
            return # if data is corrupt dont publish
        
        self.rec_ok += 1
        success_average = (self.rec_ok/(self.rec_ok+self.rec_err))*100

        rospy.logdebug(f"Pozyx read success rate: {success_average:.2f} %")        
        pose2d = Pose2D(p.x,p.y,a[2]) # angles returns list [roll,pitch,yaw] 
        self.pub_pose2d.publish(pose2d)

        period = time.time() - self.time_prev
        self.time_prev = time.time()
        
        rospy.loginfo_throttle(0.5, f"Publish 2D: \n{pose2d}\nf: {1/period} T: {period}")
        self.pub_marker(pose2d,ColorRGBA(0,1,0,1))

        return True

    def pose_flatten(self, pose_3d):
        pose_2d = Pose2D()
        
        pose_2d.x = pose_3d.position.x
        pose_2d.y = pose_3d.position.y 
        euler = euler_from_quaternion((pose_3d.orientation.x,           
                                        pose_3d.orientation.y,  
                                        pose_3d.orientation.z,  
                                        pose_3d.orientation.w))
        pose_2d.theta = euler[2] #0 roll, 1 pitch, 2 yaw
    
        return pose_2d


# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('dpr_uwb_locate')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        pb = PozyxPublish()
        pb.loop_endless()

    except rospy.ROSInterruptException:
        pb.exit_flag = True
