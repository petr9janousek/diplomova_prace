#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from enum import Enum

from dpr_measure import PozyxMeasure

class PositionMarker():
    def __init__(self):
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

    def pub_point(self, name, shape, frame, point: Point or tuple, life=1):
        """set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3"""   
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = shape
        marker.id = name
        
        # Set the scale of the marker
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Set the pose of the marker
        if type(point) is Point:
            marker.pose.position = point
        elif type(point) is tuple:
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
                
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.lifetime = rospy.Duration(life)

        rospy.logdebug(f"Marker published in: {frame}\n{point}")
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node('position_marker')
    id_counter = 1
    pm = PozyxMeasure(0x6a2c)
    while not rospy.is_shutdown():
        mark = PositionMarker()
        meas = pm.get_position()
        if meas is not None:
            mark.pub_point(id_counter, 1, "link_base", Point(meas.x, meas.y, meas.z), life=4)
            id_counter+=1