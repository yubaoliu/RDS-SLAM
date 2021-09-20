/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#ifndef _ACTION_SERVER_H_
#define _ACTION_SERVER_H_

#include "segnet.h"
#include <actionlib/server/simple_action_server.h>
#include <segnet_ros/segnetAction.h>
#include <signal.h>

namespace segnet_ros {
class SegnetActionServer {
public:
    SegnetActionServer(const ros::NodeHandle& nh, const std::string name, const std::string in_prototxt, const std::string in_caffemodel, const std::string in_lut, const string in_label_topic = std::string("/segnet/label"), const string in_label_color_topic = std::string("/segnet/label_color"));
    ~SegnetActionServer();
    void executeCB(const segnet_ros::segnetGoalConstPtr& t_goal);

private:
    ros::NodeHandle nh_;
    int frame_number_;
    actionlib::SimpleActionServer<segnet_ros::segnetAction> as_;
    std::string action_name_;
    segnet_ros::segnetFeedback feedback_;
    segnet_ros::segnetResult result_;
    SegNet* segnet_instance_;

    std::string caffemodel_;
    std::string prototxt_;
    std::string lut_;

    // publish semantic result
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_label_;
    image_transport::Publisher pub_label_color_;
};
}
#endif
