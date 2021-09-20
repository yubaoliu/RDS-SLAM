# Manually Build Probject

```sh
cd ~/catkin_ws/src/SLAM/
./build_thirdparty.sh

cd ~/catkin_ws/src
catkin_make
```

# Run SLAM client

## SegNet Version

```sh
roslaunch rds_slam tum_segnet_walk_xyz.launch
```

## MaskRCNN Version

```sh
roslaunch rds_slam tum_maskrcnn_walk_xyz.launch
```

# How to control the framerate

Due to the performance is somehow related to the GPU configuration, you can trade off the tracking performance and real-time performance by adjusting these parameters.

-   init_delay: the delay of first few frames
-   frame_delay: the delay of each frame other than the first few frames
-   init_frames: wait some time for the first few frames
