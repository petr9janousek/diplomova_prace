#! /usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from geometry_msgs.msg import Pose

rospy.init_node('gazebo_pose_republisher')
rospy.wait_for_service('gazebo/get_model_state', 10)

model_state_service_client = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
model_state_request = GetModelStateRequest()
model_state_response = GetModelStateResponse()

if __name__ == '__main__':    
    rospy.init_node('gazebo_pose_republisher')
    pub = rospy.Publisher('/decoy_pose', Pose, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            model_state_request.model_name = "actor"
            model_state_response = model_state_service_client(model_state_request) # Send through the connection the path to the trajectory file to be executed
            
            pub.publish(model_state_response.pose)
            rospy.logdebug(model_state_response.pose)
            rate.sleep()
            
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))
        except rospy.ROSInterruptException:
            exit()