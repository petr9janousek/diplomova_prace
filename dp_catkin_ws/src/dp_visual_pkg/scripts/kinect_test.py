#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import os
#import Jetson.GPIO as GPIO

class DPFaceDetector():
    def __init__(self):
        rospy.init_node('face_detector_node')
        self.image_sub = rospy.Subscriber("/kinect/depth/image_raw", Image, self.clbk_kinect)
        self.image_pub = None
        self.bridge = CvBridge()

        dirname = os.path.dirname(__file__)
        cascade_path = os.path.join(dirname,"haarcascade_frontalface.xml")
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        
        self.image = None

    #def register_publish(self):
    #    self.image_pub = rospy.Publisher('/face_detected', Bool, queue_size=1)

    def clbk_kinect(self, img_data):
            try:
                self.image = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
                self.image = cv2.resize(self.image,(320,240))   
                cv2.imshow("face",self.image)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)

    def face_detect(self, image):
        t2 = rospy.Time.now()
        #h,w = image.shape[:2]
        #print(f"h {h} w {w}")

        faces = self.face_cascade.detectMultiScale(
            image,
            scaleFactor=1.2,
            minNeighbors=3,
        )
        
        t3 = rospy.Time.now()
        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        #cv2.imshow("detecton",image)
        #cv2.waitKey(1)
        rospy.loginfo(f"{len(faces)} face detected, detect time {(t3-t2).to_sec()}")

        #if len(faces) > 0 and self.image_pub is not None:
        #    self.image_pub.publish(True)

if __name__ == "__main__":
    fd = DPFaceDetector()
    rospy.sleep(1)
    try:
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            rate.sleep()
            fd.face_detect(fd.image)
    except rospy.ROSInterruptException:
        print("exit")

        