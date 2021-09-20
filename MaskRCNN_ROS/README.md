# Overview

This is a ROS compatible version with MaskRCNN

# Reuirements

-   Ubuntu 18
-   Python 2.7
-   ROS Melodic
-   docker [optional]
-   docker-compose [optional]

# Docker

```sh
cd MaskRCNN_ROS/docker
docker-compose build
docker-compose up
```

# MaskRCNN Semantic Server

```sh
roslaunch maskrcnn_ros action_server.launch
```

# Server

## Request label and score

-   action_server.py
-   semantic.action

## Request original MaskRCNN result

-   action_server_maskrcnn.py
-   semantic_maskrcnn.action
-   objectInfo.msg 

# Get dataset

```sh
wget https://github.com/matterport/Mask_RCNN/releases/download/v2.1/balloon_dataset.zip
unzip balloon_dataset.zip
```

# CNN model

Run download_model.sh to get mask_rcnn_coco.h5 and mask_rcnn_balloon.h5.

Manually download:

```sh
wget -c https://github.com/matterport/Mask_RCNN/releases/download/v1.0/mask_rcnn_coco.h5
wget -c  https://github.com/matterport/Mask_RCNN/releases/download/v2.1/mask_rcnn_balloon.h5 
```

# Test Action communication

-   Start server
    ```sh
    roslaunch maskrcnn_ros action_server.launch 
    ```
-   Start Client

    ```sh
    roslaunch maskrcnn_ros action_client.launch
    ```

# Test MaskRCNN semantic segmentation

## Simple segmentation example

```sh
cd include/MaskRCNN/MaskRCNN/samples
python demo.py
```

## TUM dataset

```sh
cd include/MaskRCNN/MaskRCNN/samples
python tum_infer.py
```

tum_infer.py: will infer one image from TUM dataset if no argument is given
