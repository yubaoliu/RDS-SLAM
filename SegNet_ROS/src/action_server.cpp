/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#include "action_server.h"

using namespace std;

namespace segnet_ros {

SegnetActionServer::SegnetActionServer(const ros::NodeHandle& in_nh, const std::string in_action_name, const std::string in_prototxt, const std::string in_caffemodel, const std::string in_lut, const string in_label_topic, const string in_label_color_topic)
    : as_(in_nh, in_action_name, boost::bind(&SegnetActionServer::executeCB, this, _1), false)
    , action_name_(in_action_name)
    , nh_(in_nh)
    , caffemodel_(in_caffemodel)
    , prototxt_(in_prototxt)
    , lut_(in_lut)
    , it_(in_nh)
{
    LOG(INFO) << "Caffemodel: " << in_caffemodel;
    LOG(INFO) << "prototxt: " << in_prototxt;
    LOG(INFO) << "lut: " << in_lut;
    LOG(INFO) << "Action server name: " << in_action_name;

    segnet_instance_ = SegNet::getInstance();
    if (!segnet_instance_) {
        LOG(ERROR) << "Cannot create SegNet Instance";
        return;
    }
    segnet_instance_->segnetInit(in_prototxt, in_caffemodel, in_lut);
    segnet_instance_->start();

    as_.start();
    std::cout << "Starting Segnet server" << std::endl;

    // publish result
    pub_label_ = it_.advertise(in_label_topic, 10);
    pub_label_color_ = it_.advertise(in_label_color_topic, 10);

    LOG(INFO) << "======================================";
    std::cout << "============= SegNet ROS Server Ready =========================" << std::endl;
}

// Get semantic result
void SegnetActionServer::executeCB(const segnetGoalConstPtr& t_goal)
{
    if (!as_.isActive()) {
        LOG(ERROR) << "action server starting failed";
        return;
    }
    // Every request has a ID
    std::cout << "id: " << t_goal->id << std::endl;
    LOG(INFO) << "id: " << t_goal->id;

    // ROS image msg to OpenCV cv::Mat
    cv_bridge::CvImageConstPtr cv_ptrImage;
    try {
        cv_ptrImage = cv_bridge::toCvCopy(t_goal->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        feedback_.complete = false;
        as_.publishFeedback(feedback_);
        return;
    }

    // Input: RGB Image
    // Output: label and label_color
    cv::Mat label_color, label;
    //use separate thread
    segnet_instance_->SegmentMultiThread(cv_ptrImage->image, label, label_color);

    if (label.empty() || label_color.empty()) {
        feedback_.complete = false;
        as_.publishFeedback(feedback_);
        return;
    }

    // OpenCV image format to ROS msg
    sensor_msgs::ImagePtr msg_label = cv_bridge::CvImage(std_msgs::Header(), "mono8", label).toImageMsg();
    sensor_msgs::ImagePtr msg_label_color = cv_bridge::CvImage(std_msgs::Header(), "bgr8", label_color).toImageMsg();

    // Seand back result
    result_.id = t_goal->id;
    result_.label = *msg_label;
    result_.label_color = *msg_label_color;

    as_.setSucceeded(result_);
    feedback_.complete = true;
    as_.publishFeedback(feedback_);

    frame_number_++;
    std::cout << "Average time: " << segnet_instance_->m_segment_time / frame_number_ * 1000 << "ms" << std::endl;
    LOG(INFO) << "Average time: " << segnet_instance_->m_segment_time / frame_number_ * 1000 << "ms";

    // publish result
    if (pub_label_color_.getNumSubscribers()) {
        msg_label_color->header.stamp = t_goal->image.header.stamp;
        pub_label_color_.publish(msg_label_color);
    }
    if (pub_label_.getNumSubscribers()) {
        msg_label->header.stamp = t_goal->image.header.stamp;
        pub_label_.publish(msg_label);
    }
}

SegnetActionServer::~SegnetActionServer()
{
    std::cout << "----Server shutdown ----" << std::endl;
    LOG(INFO) << "----Server shutdown ----";
    segnet_instance_->stop();
    as_.shutdown();
}

}

// cv::Mat generateMask(const cv::Mat& t_label, const cv::Mat t_depth)
// {
//     cv::Mat label_mask = (t_label == 15); //15 is person in Pascal VOC
//     cv::Mat dialation, close;
//     cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
//     cv::morphologyEx(label_mask, close, CV_MOP_CLOSE, kernel);
//     cv::morphologyEx(close, dialation, CV_MOP_DILATE, kernel);
//     label_mask = dialation;
//     std::vector<vector<cv::Point>> contours;
//     cv::findContours(label_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//
//     // std::vector<cv::Point>> contours_poly(contours.size());
//     // std::vector<cv::Rect> boundRect(contours.size());
//     // std::vector<cv::Point2f> centers(contours.size());
//     // std::vector<float> radius(contours.size());
//
//     std::vector<std::vector<cv::Point>> hull(contours.size());
//     for (size_t i = 0; i < contours.size(); i++) {
//         // cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
//         // boundRect[i] = cv::boundingRect(contours_poly[i]);
//         cv::convexHull(contours[i], hull[i]);
//         cv::fillConvexPoly(label_mask, hull[i], 255);
//     }
//     cv::morphologyEx(label_mask, close, CV_MOP_CLOSE, kernel);
//     label_mask = close;
//     return label_mask; //mask
// }
