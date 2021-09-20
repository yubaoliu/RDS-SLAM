/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <assert.h>
#include <atomic>
#include <chrono>
#include <dirent.h>
#include <errno.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <iostream>
#include <mutex>
#include <string>
#include <stdlib.h>
#include <sys/stat.h>
#include <thread>
#include <utility>
#include <map> 
#include <condition_variable>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Debug
#include <glog/logging.h>

//boost
#include <boost/timer.hpp>

#endif
