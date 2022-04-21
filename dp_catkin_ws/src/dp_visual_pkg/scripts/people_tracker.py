#!/usr/bin/env python3
import os, sys
from threading import Lock

# ROS related imports
import rospy, rospkg
from sensor_msgs.msg import Image
# OpenPose objects
from openpose_ros_msgs.msg import PersonDetection, BodypartDetection

# trt_pose related imports
import json
import trt_pose.coco
import trt_pose.models
import torch

import cv2
import torchvision.transforms as transforms
import PIL.Image
from trt_pose.draw_objects import DrawObjects
from trt_pose.parse_objects import ParseObjects
#from jetcam.utils import bgr8_to_jpeg

import numpy
numpy.set_printoptions(threshold=sys.maxsize)

class PoepleTracker(object):

    def __init__(self, device_to_use="cuda", plot_images=False):
        """
        device_to_use: can be "cpu" or "cuda",
        plot_images: whether to use cv2s.imshow()
        """

        self.lock = Lock() #for camera reading and process
        self.glob=(0,0)
        rospy.on_shutdown(self.shutdownhook)

        # Variable to avoid publisher errors when closing script.
        self._device_to_use = device_to_use
        self._plot_images = plot_images

        self.peopletrack_pub = rospy.Publisher('/peopletracker/detected_skeleton', PersonDetection, queue_size=1)
        self.global_detection_pub = rospy.Publisher('/peopletracker/detected_centroid', BodypartDetection, queue_size=1)

        #PATHS DEFINES
        follow_people_configfiles_path = rospkg.RosPack().get_path('dp_visual_pkg')+"/trt_config_files"
        humanPose_file_path = os.path.join(follow_people_configfiles_path, 'human_pose.json')

        MODEL_WEIGHTS = 'resnet18_baseline_att_224x224_A_epoch_249.pth'
        #MODEL_WEIGHTS = 'densenet121_baseline_att_256x256_B_epoch_160.pth'
        model_weights_path = os.path.join(follow_people_configfiles_path, MODEL_WEIGHTS)
        
        OPTIMIZED_MODEL = 'resnet18_baseline_att_224x224_A_epoch_249_trt.pth'
        optimized_model_weights_path = os.path.join(follow_people_configfiles_path, OPTIMIZED_MODEL)

        #FROM TRT_POSE GIT DOCUMENTATION
        with open(humanPose_file_path, 'r') as f:
            self.human_pose = json.load(f)

        self.topology = trt_pose.coco.coco_category_to_topology(self.human_pose)

        num_parts = len(self.human_pose['keypoints'])
        num_links = len(self.human_pose['skeleton'])

        if device_to_use == "cuda":
            model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
        else:
            model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cpu().eval()

        if device_to_use == "cuda":
            model.load_state_dict(torch.load(model_weights_path))
        else:
            model.load_state_dict(torch.load(model_weights_path, map_location=torch.device('cpu')))

        self.WIDTH = 224 #256
        self.HEIGHT = 224 #256

        rospy.loginfo("Creating empty data")
        if device_to_use == "cuda":
            data = torch.zeros((1, 3, self.HEIGHT, self.WIDTH)).cuda()
        else:
            data = torch.zeros((1, 3, self.HEIGHT, self.WIDTH)).cpu()

        if device_to_use == "cuda":
            import torch2trt
            from torch2trt import TRTModule

            if not os.path.isfile(optimized_model_weights_path):
                rospy.loginfo("Use tortchtrt to go from Torch to TensorRT to generate an optimised model")
                self.model_trt = torch2trt.torch2trt(model, [data], fp16_mode=True, max_workspace_size=1<<25)

                rospy.loginfo("Saving new optimized model")
                torch.save(self.model_trt.state_dict(), optimized_model_weights_path)
            else:
                rospy.loginfo("Loading saved model using Torchtrt")
                self.model_trt = TRTModule()
                self.model_trt.load_state_dict(torch.load(optimized_model_weights_path))
        else:
            # CPU ONLY
            self.model_trt = model

        if device_to_use == "cuda":
            self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
            self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
        else:
            self.mean = torch.Tensor([0.485, 0.456, 0.406]).cpu()
            self.std = torch.Tensor([0.229, 0.224, 0.225]).cpu()

        self.device = torch.device(self._device_to_use)

        self.parse_objects = ParseObjects(self.topology)
        self.draw_objects = DrawObjects(self.topology)

        from cv_bridge import CvBridge, CvBridgeError

        self.cv_image = None

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_mono", Image, self.camera_callback)
            
        rospy.loginfo("Init successfully DONE.")

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            self.lock.acquire()
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.cv_image = cv2.resize(self.cv_image,(self.WIDTH,self.HEIGHT))   
            self.lock.release()
        except CvBridgeError as e:
            print(e)

    def shutdownhook(self):
        print ("Shutdown...")
        if self._plot_images:
            cv2.destroyAllWindows()

    def preprocess(self, image):
        self.device = torch.device(self._device_to_use)
        #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(self.device)
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]

    def process_image(self, image):
        data = self.preprocess(image)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        counts, objects, peaks = self.parse_objects(cmap, paf)#, cmap_threshold=0.15, link_threshold=0.15)
        # Draw New Image
        if self._plot_images:
            self.draw_objects(image, counts, objects, peaks)
            cv2.circle(image,self.glob, 1, (0,255,255), 10)
            cv2.imshow("PeopleTracking", image)
            cv2.waitKey(1)

        self.publish_skeleton_markers(object_counts=counts,
                                        objects=objects,
                                        normalized_peaks=peaks,
                                        human_pose=self.human_pose)

    def start_tracker(self):
        rospy.loginfo("Starting loop...")
        peopletracker_rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                rospy.loginfo("Processing Image...")
                before_seconds = rospy.get_time()
                self.lock.acquire()
                self.process_image(self.cv_image)
                self.lock.release()
                after_seconds = rospy.get_time()
                delta = after_seconds - before_seconds
                rospy.loginfo("GPU processed in "+str(delta))
            peopletracker_rate.sleep()
        rospy.loginfo("Terminating Loop...")


    def publish_skeleton_markers(self, object_counts, objects, normalized_peaks, human_pose):
        # We start the PersonDetection object message
        peopletrack_msgs = PersonDetection()

        keypoints = human_pose['keypoints']
        skeleton = human_pose['skeleton']

        topology = self.topology
        height = self.HEIGHT
        width = self.WIDTH

        K = topology.shape[0]
        count = int(object_counts[0])
        peopletrack_msgs.num_people_detected = count

        K = topology.shape[0]

        # represents the mean of all the detections to simplify a person detection to a point
        global_detections_num = 0
        global_x = 0
        global_y = 0
        global_bodypart = BodypartDetection()

        for i in range(count):
            peopletrack_msgs.person_ID = i

            color = (0, 255, 0)
            obj = objects[0][i]
            C = obj.shape[0]
            for j in range(C):
                # j is the index of the object in humanpose keypoints
                body_part_name = keypoints[j]
                k = int(obj[j])
                #rospy.loginfo("BodyPart="+str(body_part_name)+", k ==>"+str(k))
                if k >= 0:
                    peak = normalized_peaks[0][j][k]
                    x = round(float(peak[1]) * width)
                    y = round(float(peak[0]) * height)

                    peopletrack_msgs = self.update_peopletrack_msgs(peopletrack_msgs, body_part_name, x, y)

                    global_x += x
                    global_y += y
                    global_detections_num += 1

            self.peopletrack_pub.publish(peopletrack_msgs)
        
        if global_detections_num > 0:
            global_bodypart.x = round(float(global_x) / global_detections_num)
            global_bodypart.y = round(float(global_y) / global_detections_num)
        
        self.glob=(global_bodypart.x,global_bodypart.y)
        self.global_detection_pub.publish(global_bodypart)

    def update_peopletrack_msgs(self, peopletrack_msgs, body_part_name, x, y):

        body_part_msg = BodypartDetection()
        body_part_msg.x = x
        body_part_msg.y = y
        # If we have the data the confidence is 100%
        body_part_msg.confidence = 1.0

        # Arms
        if body_part_name == "right_shoulder":
            peopletrack_msgs.right_shoulder = body_part_msg
        if body_part_name == "right_elbow":
            peopletrack_msgs.right_elbow = body_part_msg
        if body_part_name == "right_wrist":
            peopletrack_msgs.right_wrist = body_part_msg
        if body_part_name == "left_shoulder":
            peopletrack_msgs.left_shoulder = body_part_msg
        if body_part_name == "left_elbow":
            peopletrack_msgs.left_elbow = body_part_msg
        if body_part_name == "left_wrist":
            peopletrack_msgs.left_wrist = body_part_msg

        # Legs
        if body_part_name == "right_hip":
            peopletrack_msgs.right_hip = body_part_msg
        if body_part_name == "right_knee":
            peopletrack_msgs.right_knee = body_part_msg
        if body_part_name == "right_ankle":
            peopletrack_msgs.right_ankle = body_part_msg
        if body_part_name == "left_hip":
            peopletrack_msgs.left_hip = body_part_msg
        if body_part_name == "left_knee":
            peopletrack_msgs.left_knee = body_part_msg
        if body_part_name == "left_ankle":
            peopletrack_msgs.left_ankle = body_part_msg

        # Face
        if body_part_name == "nose":
            peopletrack_msgs.nose = body_part_msg
        if body_part_name == "neck":
            peopletrack_msgs.neck = body_part_msg
        if body_part_name == "right_eye":
            peopletrack_msgs.right_eye = body_part_msg
        if body_part_name == "left_eye":
            peopletrack_msgs.left_eye = body_part_msg
        if body_part_name == "right_ear":
            peopletrack_msgs.right_ear = body_part_msg
        if body_part_name == "left_ear":
            peopletrack_msgs.left_ear = body_part_msg

        return peopletrack_msgs


if __name__ == "__main__":
    rospy.init_node("peopletracker_node")
    
    if len(sys.argv) < 3:
        rospy.logwarn("Not enough arguments provided, using defaults. Correct format: dev:cpu/gpu ros_camera:T/F plot:T/F")
        pt = PoepleTracker()
    else:
        device_to_use = sys.argv[1]
        plot_images = (sys.argv[2] == "true")

        pt = PoepleTracker(device_to_use=device_to_use,
                            plot_images=plot_images)

    pt.start_tracker()
    print("Exiting script...")