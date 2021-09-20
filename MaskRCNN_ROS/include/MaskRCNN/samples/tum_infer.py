import os
import sys
import random
import math
import numpy as np
import skimage.io
import cv2
import matplotlib
import matplotlib.pyplot as plt

# Root directory of the project
ROOT_DIR = os.path.abspath("../")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
# from mrcnn import utils
import mrcnn.model as modellib
from mrcnn import visualize
from mrcnn import utils
# Import COCO config
sys.path.append(os.path.join(ROOT_DIR,
                             "samples/coco/"))  # To find local version
import coco

# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Local path to trained weights file
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "model/mask_rcnn_coco.h5")
# Download COCO trained weights from Releases if needed
# if not os.path.exists(COCO_MODEL_PATH):
# utils.download_trained_weights(COCO_MODEL_PATH)

# Directory of images to run detection on
IMAGE_DIR = os.path.join(ROOT_DIR, "images")

file_names = 'images/tum_rgb.png'
if len(sys.argv) > 1:
    file_names = sys.argv[1]
# file_names = 'results/0.png'


class InferenceConfig(coco.CocoConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1


config = InferenceConfig()
config.display()

# Create model object in inference mode.
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

# Load weights trained on MS-COCO
model.load_weights(COCO_MODEL_PATH, by_name=True)

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

# Load a random image from the images folder
# file_names = next(os.walk(IMAGE_DIR))[2]
# image = skimage.io.imread(os.path.join(IMAGE_DIR, random.choice(file_names)))

image = skimage.io.imread(os.path.join(ROOT_DIR, file_names))

# Use opencv to reead file
# image = cv2.imread(os.path.join(ROOT_DIR, file_names))
print(image.shape)

# Run detection
results = model.detect([image], verbose=1)

# Visualize results
r = results[0]
# visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'],
# class_names, r['scores'])

rois = r['rois']
masks = r['masks']
class_ids = r['class_ids']
scores = r['scores']

N = rois.shape[0] 

print("ROIS: ", rois.shape)
print("masks: ", masks.shape)
print("class ID: ", class_ids.shape)
print("scores:", scores.shape)

print("Class 0: ", class_ids[0])
print("0-> score: ", scores[0])
print("0-> mask: ", masks[:, :, 0].shape)
print("0-> rois: ", rois[:,0].shape)

for i in range(N):
    print('=========== %d ========' %i)
    mask = masks[:,:,i]
    print(mask)
    print("mask: ", mask.shape)
    # print('mask as uint8', mask.astype(uint8))
    print('type: ', mask.dtype)
    file_name = 'mask' + str(i) + '.png'
    cv2.imwrite(file_name, mask*255)
    

