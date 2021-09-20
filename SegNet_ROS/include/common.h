/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#ifndef _COMMON_H_
#define _COMMON_H_

//CPP
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <stdlib.h>

//OpenCV
#include <opencv2/core/core.hpp>

//ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>

//Glog
#include <glog/logging.h>

//boost 
#include <boost/thread/thread.hpp>

#endif
