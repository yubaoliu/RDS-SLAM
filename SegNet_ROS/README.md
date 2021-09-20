# Introduction

This is a ROS compatible version of SegNet.

# Kick off SegNet Server

```sh
roslaunch segnet_ros action_server.launch
```

# caffe model

-   segnet_pascal.caffemodel

<https://github.com/alexgkendall/SegNet-Tutorial/blob/master/Example_Models/segnet_model_zoo.md>

# demo

```sh
roslaunch segnet_ros action_server.launch

roslaunch segnet_ros action_client.launch
```
