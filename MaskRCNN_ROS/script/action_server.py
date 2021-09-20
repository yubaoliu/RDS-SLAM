#!/usr/bin/env python
"""
\author Yubao Liu
\date Dec. 2020
"""
from __future__ import division
from __future__ import print_function


import mrcnn.model as modellib
from mrcnn import visualize
from mrcnn import utils
from tensorflow.python.keras.models import load_model
from tensorflow.python.keras.backend import set_session
from maskrcnn_ros.msg import *
import maskrcnn_ros.msg
import actionlib
import PIL
import matplotlib.pyplot as plt
import matplotlib
import skimage.io
import time
import logging
import threading
import copy
import cv2
from skimage.transform import resize
import message_filters
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import rospy
import math
import random
import sys

import os
os.environ["CUDA_VISIBLE_DEVICES"]="1"

import tensorflow as tf

config = tf.ConfigProto()
# config.gpu_options.per_process_gpu_memory_fraction=0.8
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)


# Here should be the project name

# logging.basicConfig( level=logging.DEBUG, format='(%(threadName)-9s) %(message)s',)

logging.basicConfig(filename='maskrcnn_action_server.log',
                    format='%(asctime)s %(levelname)s:%(message)s',
                    level=logging.INFO,
                    datefmt='%m/%d/%Y %I:%M:%S %p')

# Maskrcnn
# Root directory of the project
# TODO: Set ROOT_DIR in launch file
ROOT_DIR = os.path.abspath(
    "/root/catkin_ws/src/MaskRCNN_ROS/include/MaskRCNN/")
# ROOT_DIR = os.path.abspath( "/root/catkin_ws/src/MaskRCNN_ROS/include/MaskRCNN/")
print("ROOT_DIR: ", ROOT_DIR)

# why cannot use the relative path
# ROOT_DIR = os.path.abspath("include/MaskRCNN/")
# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
# from mrcnn import utils
# Import COCO config
# To find local version
sys.path.append(os.path.join(ROOT_DIR, "samples/coco/"))
import coco


# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")
# Local path to trained weights file
# COCO_MODEL_PATH = os.path.join(ROOT_DIR, "model/mask_rcnn_coco.h5")
# Directory of images to run detection on
IMAGE_DIR = os.path.join(ROOT_DIR, "images")
# RESULT_DIR = os.path.join(ROOT_DIR, "results")

# COCO Class names
# Index of the class in the list is its ID. For example, to get ID of
# the teddy bear class, use: class_names.index('teddy bear')
class_names = [
    'BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
    'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
    'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
    'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
    'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
    'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
    'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush'
]

# Totoal time consuming
total_time = float(0.0)
# Total number of images
total_number = int(0)

# To sync semantic segmentation thread and action server thread
cn_task = threading.Condition()
cn_ready = threading.Condition()

batch_size = 2

class InferenceConfig(coco.CocoConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 2
    IMAGE_MIN_DIM = 128
    IMAGE_MAX_DIM = 128


class MaskRcnn(object):
    def __init__(self):
        print('Init MaskRCNN')
        # Init Mask rcnn
        # Initialize Maskrcnn
        config = InferenceConfig()
        config.display()
        # Create model object in inference mode.
        self.model = modellib.MaskRCNN(mode="inference",
                                       model_dir=MODEL_DIR,
                                       config=config)
        self.coco_model_path_ = rospy.get_param(
            'model_path', '/root/cnnmodel/mask_rcnn_coco.h5')
        # Load weights trained on MS-COCO
        # self.model.load_weights(COCO_MODEL_PATH, by_name=True)
        self.model.load_weights(self.coco_model_path_, by_name=True)

        # if bPublish_result:
        self.masked_image__pub = rospy.Publisher("masked_image",
                                                 Image,
                                                 queue_size=10)

        print("=============Initialized MaskRCNN==============")

    def segment(self, image):
        timer_start = rospy.Time.now()

        results = self.model.detect(image, verbose=1)

        segment_time = (rospy.Time.now() - timer_start).to_sec() * 1000
        print("predict time: %f ms \n" % segment_time)

        timer_start = rospy.Time.now()
        r = results[0]
        self.masked_image_ = visualize.ros_semantic_result(image[0],
                                                           r['rois'],
                                                           r['masks'],
                                                           r['class_ids'],
                                                           class_names,
                                                           r['scores'],
                                                           show_opencv=False)
        segment_time = (rospy.Time.now() - timer_start).to_sec() * 1000
        print("Visualize time: %f ms \n" % segment_time)

        return self.masked_image_, results
        # visualize.display_instances(color_image, r['rois'], r['masks'], r['class_ids'], class_names, r['scores'])

    def publish_result(self, image, stamp):
        if image is None or stamp is None:
            print("Image invalid")
            return
        # h, w, c = image.shape
        # assert c == 3
        msg_img = CvBridge().cv2_to_imgmsg(image, encoding='passthrough')
        msg_img.header.stamp = stamp
        self.masked_image__pub.publish(msg_img)


# Segmentation cannot putted into ROS action callback
def worker(maskrcnn, bPublish_result=False):
    # Control whether start maskrcnn thread
    global bStartMaskRCNN
    global color_image
    global stamp
    global masked_image
    global result

    maskrcnn = MaskRcnn()
    bStartMaskRCNN = True
    color_image = []
    stamp = []
    while bStartMaskRCNN:
        with cn_task:
            cn_task.wait()
            print("New task comming")
            timer_start = rospy.Time.now()
            masked_image, result = maskrcnn.segment(color_image)
            segment_time = (rospy.Time.now() - timer_start).to_sec() * 1000
            print("MaskRCNN segment time for cuurent image: %f ms \n" %
                  segment_time)
            with cn_ready:
                cn_ready.notifyAll()
                if bPublish_result:
                    maskrcnn.publish_result(masked_image, stamp)

    print("Exit MaskRCNN thread")


class SemanticActionServer(object):
    _feedback = maskrcnn_ros.msg.batchFeedback()
    _result = maskrcnn_ros.msg.batchResult()

    def __init__(self):
        print("Initialize Action Server")

        # Get action server name
        self._action_name = rospy.get_param('/semantic/action_name',
                                            '/maskrcnn_action_server')
        print("Action name: ", self._action_name)

        # Start Action server
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            maskrcnn_ros.msg.batchAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self._as.start()

        # Semantic segentation result
        # self.label_topic = rospy.get_param('/semantic/semantic_label',
        #                                    '/semantic_label')
        # self.label_color_topic = rospy.get_param(
        #     '/semantic/semantic_label_color', '/semantic_label_color')

        # publish result
        # self.semantic_label_pub = rospy.Publisher(self.label_topic,
        #                                           Image,
        #                                           queue_size=10)
        # self.semantic_label_color_pub = rospy.Publisher(self.label_color_topic,
        #                                                 Image,
        #                                                 queue_size=10)

        # self.graph = tf.get_default_graph()
        # image = cv2.imread('/root/catkin_ws/src/MaskRCNN_ROS/include/MaskRCNN/images/tum_rgb.png')
        # results = self.model.detect([image], verbose=1)
        # r = results[0]
        # visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], class_names, r['scores'])

        print("MaskRCNN action server start...")

    # Notice: Inconvinient to add MaskRCNN detection here, because this callback running in a seperate thread,
    # The initializaiotn of MaskRCNN will not work here
    def execute_cb(self, goal):
        global total_time
        global total_number
        global color_image
        global stamp
        global masked_image
        global result
        # global bSaveResult

        # clear the old data
        color_image = []
        stamp = []

        if not self._as.is_active():
            logging.debug("[Error] MaskRCNN action server cannot active")
            return

        time_each_loop_start = rospy.Time.now()

        print("----------------------------------")
        print("ID: %d" % goal.id)
        # batch_size = goal.batch_size
        # print("batch size: %d" %  batch_size)

        # Receive source image from client
        for i in range(batch_size):
            try:
                color_image.append(
                    CvBridge().imgmsg_to_cv2(goal.image[i], 'bgr8'))
                stamp.append(goal.image[i].header.stamp)
                # logging.debug('Color: ', color_image.shape)
            except CvBridgeError as e:
                print(e)
                return

        # Test image communication
        # (rows, cols, channels) = color_image[0].shape
        # print("rows: ", rows, " cols: ", cols)
        # verify the image received
        # cv2.imshow("Image window", color_image)
        # cv2.waitKey(1)

        timer_start = rospy.Time.now()
        # perform segmenting
        if np.any(color_image[0]):
            with cn_task:
                print("Inform new task")
                cn_task.notifyAll()

            with cn_ready:
                cn_ready.wait()
                print("semantic result ready")

        # save_file = RESULT_DIR +'/'+ str(goal.id) + '.png'
        # cv2.imwrite(save_file, color_image)
        # skimage_image = skimage.io.imread(save_file)
        # print("Shape of skimage: ")

        # Visualize results
        # r = results[0]
        # visualize.display_instances(color_image, r['rois'], r['masks'], r['class_ids'], class_names, r['scores'])
        # # to ROS msg
        # msg_label_color = CvBridge().cv2_to_imgmsg(decoded, encoding="bgr8")
        # msg_label = CvBridge().cv2_to_imgmsg(label, encoding="mono8")

        # calculate time cost
        segment_time = (rospy.Time.now() - timer_start).to_sec() * 1000
        print("MaskRCNN segment time: %f ms \n" % segment_time)
        # calculate average time consuming
        total_time = float(total_time) + float(segment_time)
        total_number = total_number + 1
        if int(total_number) > 0:
            average = total_time / float(total_number)
            print("Average time: %f ms" % average)

        # object_num = []
        labelMsgs = []
        # scoreMsgs = []

        for i in range(batch_size):
            # Original results of MaskRCNN
            boxes = result[i]['rois']
            masks = result[i]['masks']
            class_ids = result[i]['class_ids']
            # scores = result[i]['scores']

            # Total number of objects
            N = boxes.shape[0]

            # Result of request
            label_img = np.zeros(masks.shape[0:2], dtype=np.uint8)
            #  score_img = np.zeros(masks.shape[0:2], dtype=np.float32)
            logging.info('shap of label image: %s', label_img.shape)

            # label_img_color = np.zeros((masks.shape[0], masks.shape[1], 3), dtype=np.uint8)
            # logging.info('shap of label_color image: %s', label_img_color.shape)
            for i in range(N):
                # masks is 0/1 two value image, extents to [0-255]
                mask = masks[:, :, i] * 255
                # print('type of mask: ', mask.dtype)
                # merge mask of each object to one mask image
                label_img += (masks[:, :, i] * class_ids[i]).astype(np.uint8)
                #  score_img += (masks[:, :, i] * scores[i]).astype(np.float32)

            # semantic label
            msg_label = CvBridge().cv2_to_imgmsg(label_img.astype(np.uint8), encoding='mono8')
            #  msg_score = CvBridge().cv2_to_imgmsg(
                #  score_img.astype(np.float32), encoding='passthrough')

            labelMsgs.append(msg_label)
            # scoreMsgs.append(msg_score)
            # object_num.append(N)

        # return the request result
        # Total number of objects
        # self._result.object_num = object_num
        # Return segment result
        self._result.id = goal.id
        self._result.label = labelMsgs
        #  self._result.score = scoreMsgs

        # feedback
        self._feedback.complete = True
        self._as.set_succeeded(self._result)
        self._as.publish_feedback(self._feedback)

        # calculate time consuming
        time_each_loop = (rospy.Time.now() -
                          time_each_loop_start).to_sec() * 1000
        print("Time of each request: %f ms \n" % time_each_loop)

        # Save results after sending back the results
        # if bSaveResult:
        # boxes = result['rois']
        # masks = result['masks']
        # class_ids = result['class_ids']
        # scores = result['scores']
        # N = boxes.shape[0]
        # for i in range(N):
        #     save_file = RESULT_DIR + '/mask/' + str(goal.id) + '_' + str(class_ids[i]) + '.png'
        #     m_img = (masks[:, :, i] * 255).astype(np.uint8)
        #     cv2.imwrite(save_file, m_img)
        #     # print('type of m_img: ', m_img.dtype)

        # # Save semantic label
        # save_file = RESULT_DIR + '/label/' + str(goal.id) + '.png'
        # cv2.imwrite(save_file, label_img)
        # print('type of label_img: ', label_img.dtype)

        # publish result
        # masked_image: just for visualization
        # msg_label_color = CvBridge().cv2_to_imgmsg(masked_image, encoding="bgr8")

        # if self.semantic_label_pub.get_num_connections() > 0:
        #     msg_label.header.stamp = goal.image.header.stamp
        #     self.semantic_label_pub.publish(msg_label)
        #
        # if self.semantic_label_color_pub.get_num_connections() > 0:
        #     msg_label_color.header.stamp = goal.image.header.stamp
        #     self.semantic_label_color_pub.publish(msg_label_color)


def main(args):
    global total_time
    global total_number
    # global bSaveResult

    # save result
    # bSaveResult = True
    # bSaveResult = False

    rospy.init_node('maskrcnn_server', anonymous=False)

    th_maskrcnn = threading.Thread(name='maskrcnn',
                                   target=worker,
                                   args=(
                                       MaskRcnn,
                                       False,
                                   ))
    th_maskrcnn.start()

    actionserver = SemanticActionServer()

    print('Setting up MaskRCNN Action Server...')

    logging.debug('Waiting for worker threads')
    bStartMaskRCNN = False
    main_thread = threading.currentThread()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    for t in threading.enumerate():
        if t is not main_thread:
            t.join()


if __name__ == '__main__':
    main(sys.argv)
