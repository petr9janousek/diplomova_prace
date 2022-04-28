#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from threading import Lock
import time

#import Jetson.GPIO as GPIO

class DPFaceDetector():
    def __init__(self, show=False):
        rospy.init_node('face_detector_node')
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_rect_mono", Image, self.clbk_kinect)
        self.depth_sub = rospy.Subscriber("/kinect/depth/image_rect", Image, self.clbk_kinect_depth)
        self.image_pub = None
        self.bridge = CvBridge()

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.image = None
        self.depth = None

        self.read_lock = Lock()
        self.read_depth_lock = Lock()


        self._show_img = show

    def clbk_kinect_depth(self,img_data):
        try:
            self.read_depth_lock.acquire()
            self.depth = self.bridge.imgmsg_to_cv2(img_data, desired_encoding='passthrough')
            if self.depth is not None:
                self.depth = cv2.resize(self.depth,(320,240))
            self.read_depth_lock.release()
            #if self._show_img:
            #        cv2.imshow("depth",depth)
            #        cv2.waitKey(1)
        except CvBridgeError as e:
            self.image = None
            rospy.logwarn(e)

    def clbk_kinect(self, img_data):
        try:
            self.read_lock.acquire()
            self.image = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="mono8")
            if self.image is not None:
                self.image = cv2.resize(self.image,(320,240))   
            self.read_lock.release()
            #if self._show_img:
            #        cv2.imshow("face",self.image)
            #        cv2.waitKey(1)
        except CvBridgeError as e:
            self.image = None
            rospy.logwarn(e)

    def body_detect(self):
        #h,w = image.shape[:2]
        #print(f"h {h} w {w}")
        self.read_lock.acquire()
        start = time.perf_counter()
        boxes, weights = self.hog.detectMultiScale(self.image, winStride=(8,8))
        end = time.perf_counter()
        self.read_lock.release()

        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
        
        self.read_lock.acquire()
        if self._show_img:
            for (x1, y1, x2, y2) in boxes:
                x = int(x1+(x2-x1)/2)
                y = int(y1+(y2-y1)/2)
                
                self.read_depth_lock.acquire()
                dist = np.median(self.depth[x-10:x+10,y-10:y+10])
                self.read_depth_lock.release()

                cv2.circle(self.image,(x,y), 1, (127,127,127), 10)
                cv2.line(self.image, (160,0), (160,240), (127,127,127), 1)
                cv2.line(self.image, (160,y), (x,y), (255,255,255), 1)
                cv2.putText(self.image, f"{(y-160)}, {dist}", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255,255), 1, cv2.LINE_AA)
                #cv2.rectangle(image, (x1, y1), (x2, y2),(255, 255, 0),1)    

            cv2.imshow("detector",self.image)
            cv2.waitKey(1)
            self.read_lock.release()
        print(f"{len(boxes)}, {end-start}")

if __name__ == "__main__":
    fd = DPFaceDetector(show=True)
    rospy.sleep(1)
    try:
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            rate.sleep()
            if fd.image is not None and fd.depth is not None:
                fd.body_detect()

    except rospy.ROSInterruptException:
        print("exit")

        