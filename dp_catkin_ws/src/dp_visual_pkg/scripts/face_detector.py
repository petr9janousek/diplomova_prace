#!/usr/bin/env python3

import rospy, rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import os, time
from threading import Lock
import numpy as np

#import Jetson.GPIO as GPIO

haar_cascade_dict = {
    "face" : "haarcascade_frontalface_default.xml",
    "eye" : "haarcascade_eye.xml",
    "profile" : "haarcascade_profileface.xml",
    "lowerbody" : "haarcascade_lowerbody.xml",
    "smile" : "haarcascade_smile.xml",
    "face_alt" : "haarcascade_frontalface_alt.xml",
}

class DPFaceDetector():
    def __init__(self, device, show, detect="face"):
        rospy.init_node('face_detector_node')

        self.image_sub = rospy.Subscriber("/kinect/rgb/image_rect_mono", Image, self.clbk_kinect)
        self.depth_sub = rospy.Subscriber("/kinect/depth/image_rect", Image, self.clbk_kinect_depth)
        self.image_pub = None
        self.bridge = CvBridge()
        self.read_lock = Lock()
        self.read_depth_lock = Lock()

        self._device = device
        self._show_img = show
        
        dirname = rospkg.RosPack().get_path('dp_visual_pkg')+"/haar_config_files"

        if self._device == "cuda":
            dirname = os.path.join(dirname,"cuda")
        else:
            dirname = os.path.join(dirname,"default")

        
        assert detect in haar_cascade_dict
        
        cascade_path = os.path.join(dirname, haar_cascade_dict[detect])
        
        rospy.loginfo(f"Searching for cascades in path: {dirname}")
        rospy.loginfo(f"The file is valid: {os.path.isfile(cascade_path)}")
        rospy.loginfo(f"Detecting using {haar_cascade_dict[detect]}")

        if self._device == "cuda":
            self.face_cascade = cv2.cuda_CascadeClassifier.create(cascade_path)
        else:
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
        
        assert not self.face_cascade.empty()
        
        self.image = None
        self.depth = None

    #def register_publish(self):
    #    self.image_pub = rospy.Publisher('/face_detected', Bool, queue_size=1)
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

    def face_detect(self):
        self.read_lock.acquire()
        #h,w = image.shape[:2]
        #print(f"h {h} w {w}")
        faces=()
        start = time.perf_counter()
        if self._device == "cuda":
            cuFrame = cv2.cuda_GpuMat(self.image)
            faces = self.face_cascade.detectMultiScale(     
            cuFrame,
            scaleFactor=1.2,
            minNeighbors=3
        )
        else:
            faces = self.face_cascade.detectMultiScale(
                self.image,
                scaleFactor=1.2,
                minNeighbors=3
            )
        end = time.perf_counter()

        if self._show_img:
            for (x, y, w, h) in faces:
                cw = x+int(w/2)
                ch = y+int(h/2)
                self.read_depth_lock.acquire()
                dist = np.median(self.depth[x:x+w,y:y+h])
                self.read_depth_lock.release()

                cv2.rectangle(self.image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.line(self.image, (160,0), (160,240), (127,127,127), 1)
                cv2.line(self.image, (160,ch), (cw,ch), (255,255,255), 1)
                cv2.putText(self.image, f"{(cw-160)}, {dist}", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255,255), 1, cv2.LINE_AA)
            #cv2.putText(self.image, str(self.depth[10,10]), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255,255), 1, cv2.LINE_AA)
            cv2.imshow("detector",self.image)
            cv2.waitKey(1)

        self.read_lock.release()

        print(f"{len(faces)}, {end-start}")

        #if len(faces) > 0 and self.image_pub is not None:
        #    self.image_pub.publish(True)

if __name__ == "__main__":
    fd = DPFaceDetector("cpu", True, "face")
    rospy.sleep(1)
    try:
        rate = rospy.Rate(4)
        cont = True
        while not rospy.is_shutdown():
            rate.sleep()
            if fd.image is not None and fd.depth is not None and cont:
                fd.face_detect()

            key = cv2.waitKey(30) 
            if key == ord('s'):
                cont = False
            elif key == ord('g'):
                cont = True
    except rospy.ROSInterruptException:
        print("exit")

        