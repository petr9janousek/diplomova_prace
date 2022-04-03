#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

from dp_uwb_alg1 import Positioning_Alg1
from dp_uwb_imu import IMUsensor
from dp_uwb_utils import connect_device

class DP_UWB_Positioning():
    
    def __init__(self):
        self.position_publisher = rospy.Publisher('/uwb/position/pose', Pose, queue_size=1)
        self.pozyx = connect_device()
        if self.pozyx is None:
            rospy.logfatal("Unable to connect POZYX")
            quit()

        self.position_alg = Positioning_Alg1(self.pozyx)
        self.imu_sensor = IMUsensor(self.pozyx)

        self.pose = Pose()
        self.err_count = 0 #can init to 0 as pos_count always is at least 1 in publish pos -> 1/1 -> no /0 exception
        self.pos_count = 0 #can init to 0 as pos_count always is at least 1 -> 1/1 -> no /0 exception
        self.exit_flag = False
        rospy.on_shutdown(self.shutdownhook)
        
    def get_position(self):
        coords = self.position_alg.get_position()
        rospy.loginfo(coords)
        return coords
    
    def get_quaternion(self):
        quat = self.imu_sensor.get_quaterion()
        rospy.loginfo(quat)
        return quat

    def publish_pose(self):
        hz_rate = rospy.Rate(10)
        while not self.exit_flag:
            coords = self.get_position()
            quat = self.get_quaternion()
            if coords is not None and quat is not None:
                self.position_publisher.publish(Point(coords.x, coords.y, coords.z),
                                                Quaternion(quat.x,quat.y,quat.z,quat.w))
                self.pos_count+=1
                avg = (self.pos_count/(self.pos_count+self.err_count))*100
                rospy.loginfo(f"Pozyx doPositioning pose published. Success rate {avg:.2f} %")
            else:
                self.err_count+=1    
            hz_rate.sleep()
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.exit_flag = True
            
if __name__ == '__main__':
    rospy.init_node('uwb_position_pose')
      
    pos = DP_UWB_Positioning()
    
    try:
        pos.publish_pose()
    except rospy.ROSInterruptException:
        pass
