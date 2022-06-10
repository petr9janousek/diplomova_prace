#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

from dp_uwb_alg1 import Positioning_Alg1
from dp_uwb_imu import IMUsensor
from dp_uwb_utils import (connect_device)

from dp_uwb_markers import PositionMarker

class DP_UWB_Positioning():
    
    def __init__(self):
        self.position_publisher = rospy.Publisher('/uwb/position/Point', Point, queue_size=1)
        self.pozyx = connect_device()
        if self.pozyx is None:
            rospy.logfatal("Unable to connect POZYX")
            quit()

        self.position_alg = Positioning_Alg1(self.pozyx, remote_id=0x6a2c, publish=True)

        self.marker = PositionMarker()
        
        self.coords = Point()
        self.err_count = 0 #can init to 0 as pos_count always is at least 1 in publish pos -> 1/1 -> no /0 exception
        self.pos_count = 0 #can init to 0 as pos_count always is at least 1 -> 1/1 -> no /0 exception
        self.exit_flag = False
        rospy.on_shutdown(self.shutdownhook)

    def pub_markers(self):
        self.marker.pub_point(2,1,"link_base",Point(-0.4+0.2,-0.3,1.1)) #0.224=boxlength/2
        self.marker.pub_point(3,1,"link_base",Point(0+0.2,   -0.3,1.1))
        self.marker.pub_point(4,1,"link_base",Point(0+0.2,    0.3,1.1))
        self.marker.pub_point(5,1,"link_base",Point(-0.4+0.2, 0.3,1.1))
        
    def get_position(self):
        coords = self.position_alg.get_position()
        rospy.loginfo(coords)
        return coords
    
    def publish_position(self):
        hz_rate = rospy.Rate(20)
        while not self.exit_flag:
            coords = self.get_position()
            if coords is not None:
                p = Point(coords.x, coords.y, coords.z)
                self.position_publisher.publish(p)
                self.pos_count+=1
                avg = (self.pos_count/(self.pos_count+self.err_count))*100
                self.pub_markers()
                self.marker.pub_point(1,1,"link_base",p)
                rospy.loginfo(f"Pozyx doPositioning coordinates published. Success rate {avg:.2f} %")
            else:
                self.err_count+=1    
            hz_rate.sleep()
    
    def publish_pose(self):
        hz_rate = rospy-Rate(20)
        while not self.exit_flag():
            coor
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.exit_flag = True
            
if __name__ == '__main__':
    rospy.init_node('uwb_position_alg1')
      
    pos = DP_UWB_Positioning()

    try:
        pos.publish_position()
    except rospy.ROSInterruptException:
        pos.shutdownhook()
