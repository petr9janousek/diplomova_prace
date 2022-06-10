#!/usr/bin/env python
import rospy, tf
from geometry_msgs.msg import Point, Quaternion

from dp_uwb_alg1 import Positioning_Alg1
from dp_uwb_imu import IMUsensor
from dp_uwb_utils import connect_device

class DP_UWB_Positioning():
    
    def __init__(self):
        self.pozyx = connect_device()
        if self.pozyx is None:
            rospy.logfatal("Unable to connect POZYX")
            quit()

        self.position_alg = Positioning_Alg1(self.pozyx, remote_id=0x6a5f)
        self.imu_sensor = IMUsensor(self.pozyx, remote_id=0x6a5f)

        self.br = tf.TransformBroadcaster()
        
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

    def broadcast_tf(self):
        hz_rate = rospy.Rate(10)
        while not self.exit_flag:
            coords = self.get_position()
            quat = self.get_quaternion()
            if coords is not None and quat is not None:
                self.br.sendTransform((coords.x,coords.y,coords.z),
                                     (quat.x,quat.y,quat.z,quat.w),
                                     rospy.Time.now(),
                                     "/imu",
                                     "/world")
                
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
        pos.broadcast_tf()
    except rospy.ROSInterruptException:
        pass
