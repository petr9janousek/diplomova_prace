#!/usr/bin/env python3

import rospy, rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import os
from threading import Lock

#import Jetson.GPIO as GPIO

class DPFaceDetector():
    def __init__(self, device, show):
        rospy.init_node('face_detector_node')

        self.image_sub = rospy.Subscriber("/kinect/rgb/image_mono", Image, self.clbk_kinect)
        self.image_pub = None
        self.bridge = CvBridge()
        self.read_lock = Lock()

        self._device = device
        self._show_img = show
        
        dirname = rospkg.RosPack().get_path('dp_visual_pkg')+"/haar_config_files"

        if self._device == "cuda":
            dirname = os.path.join(dirname,"cuda")
        else:
            dirname = os.path.join(dirname,"default")

        cascade_path = os.path.join(dirname,"haarcascade_lowerbody.xml")

        rospy.loginfo(f"Searching for cascades in path: {dirname}")
        rospy.loginfo(f"The file is valid: {os.path.isfile(cascade_path)}")

        if self._device == "cuda":
            self.face_cascade = cv2.cuda_CascadeClassifier.create(cascade_path)
        else:
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
        
        assert not self.face_cascade.empty()
        
        self.image = None

    #def register_publish(self):
    #    self.image_pub = rospy.Publisher('/face_detected', Bool, queue_size=1)

    def clbk_kinect(self, img_data):
            try:
                self.read_lock.acquire()
                self.image = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="mono8")
                self.image = cv2.resize(self.image,(320,240))   
                self.read_lock.release()
                if self._show_img:
                    cv2.imshow("face",self.image)
                    cv2.waitKey(1)
            except CvBridgeError as e:
                self.image = None
                print(e)

    def face_detect(self):
        self.read_lock.acquire()
        t2 = rospy.Time.now()
        #h,w = image.shape[:2]
        #print(f"h {h} w {w}")
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
        t3 = rospy.Time.now()

        for (x, y, w, h) in faces:
            cv2.rectangle(self.image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        if self._show_img:
            cv2.imshow("detecton",self.image)
            cv2.waitKey(1)

        self.read_lock.release()
        rospy.loginfo(f"{len(faces)} face detected, detect time {(t3-t2).to_sec()}")

        #if len(faces) > 0 and self.image_pub is not None:
        #    self.image_pub.publish(True)

if __name__ == "__main__":
    fd = DPFaceDetector("cpu", True)
    rospy.sleep(1)
    try:
        rate = rospy.Rate(4)
        cont = True
        while not rospy.is_shutdown():
            rate.sleep()
            if fd.image is not None and cont:
                fd.face_detect()

            key = cv2.waitKey(30) 
            if key == ord('s'):
                cont = False
            elif key == ord('g'):
                cont = True
    except rospy.ROSInterruptException:
        print("exit")

        