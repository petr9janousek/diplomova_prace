#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import numpy as np
from sensor_msgs.msg import Image

#import Jetson.GPIO as GPIO

class DPFaceDetector():
    def __init__(self):
        rospy.init_node('face_detector_node')
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_mono", Image, self.clbk_kinect)
        self.image_pub = None
        self.bridge = CvBridge()

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.image = None

    def clbk_kinect(self, img_data):
            try:
                self.image = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
                self.image = cv2.resize(self.image,(320,240))   
                cv2.imshow("face",self.image)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)

    def body_detect(self, image):
        t2 = rospy.Time.now()
        #h,w = image.shape[:2]
        #print(f"h {h} w {w}")

        boxes, weights = self.hog.detectMultiScale(image, winStride=(8,8))
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
        
        for (x1, y1, x2, y2) in boxes:
            x = int(x1+(x2-x1)/2)
            y = int(y1+(y2-y1)/2)
            
            cv2.circle(image,(x,y), 1, (0,255,255), 10)
            cv2.rectangle(image, (x1, y1), (x2, y2),(255, 255, 0),1)    

        t3 = rospy.Time.now() 

        cv2.imshow("detecton",image)
        cv2.waitKey(1)
        rospy.loginfo(f"{len(boxes)} body detected, detect time {(t3-t2).to_sec()}")

if __name__ == "__main__":
    fd = DPFaceDetector()
    rospy.sleep(1)
    try:
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            rate.sleep()
            fd.body_detect(fd.image)
    except rospy.ROSInterruptException:
        print("exit")

        