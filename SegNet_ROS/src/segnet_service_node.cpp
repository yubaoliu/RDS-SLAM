/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#include "segnet.h"

#include <actionlib/server/simple_action_server.h>
#include <segnet_ros/semanticSegment.h>
#include <segnet_ros/segnetAction.h>
#include <signal.h>

using namespace std;
using namespace segnet_ros;

// image_transport::Publisher pub_label_color_service;
image_transport::Publisher pub_label_color;
// image_transport::Publisher pub_color;
// image_transport::Publisher pub_depth;
image_transport::Publisher pub_label;
// image_transport::Publisher pub_mask;

ros::Subscriber rgb_sub;
ros::Subscriber depth_sub;
int frame_number;
int mode;

std::string caffemodel;
std::string prototxt;
std::string lut;
class SemanticActionServer;

SemanticActionServer* server;
// ros::ServiceServer* service;
SegNet* segnet_instance;

class SemanticActionServer {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<segnetAction> as_;
    std::string action_name_;
    segnetFeedback feedback_;
    segnetResult result_;

public:
    SemanticActionServer(ros::NodeHandle& nh, std::string name)
        : as_(nh, name, boost::bind(&SemanticActionServer::executeCB, this, _1), false)
        , action_name_(name)
        , nh_(nh)
    {
        std::cout << "Start server" << std::endl;
        // as_.registerPreemptCallback(boost::bind(&SemanticActionServer::preemptCB, this));
        as_.start();
    }

    // void preemptCB()
    // {
    //     ROS_INFO("Preempted");
    // }

    void executeCB(const segnetGoalConstPtr& t_goal)
    {
        if (!as_.isActive())
            return;
        if (!segnet_instance)
            return;

        cv_bridge::CvImageConstPtr cv_ptrImage;
        std::cout << "id: " << t_goal->id << std::endl;
        try {
            cv_ptrImage = cv_bridge::toCvCopy(t_goal->image);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            feedback_.complete = false;
        }

        cv::Mat label_color, label;
        //use separate thread
        segnet_instance->segment(cv_ptrImage->image, label, label_color);

        if (label.empty() || label_color.empty()) {
            feedback_.complete = false;
            as_.publishFeedback(feedback_);
        }

        sensor_msgs::ImagePtr msg_label = cv_bridge::CvImage(std_msgs::Header(), "mono8", label).toImageMsg();
        sensor_msgs::ImagePtr msg_label_color = cv_bridge::CvImage(std_msgs::Header(), "bgr8", label_color).toImageMsg();

        result_.id = t_goal->id;
        result_.label = *msg_label;
        result_.label_color = *msg_label_color;

        as_.setSucceeded(result_);
        feedback_.complete = true;
        as_.publishFeedback(feedback_);

        frame_number++;
        std::cout << "Average time: " << segnet_instance->m_segment_time / frame_number * 1000 << "ms" << std::endl;

        if (pub_label_color.getNumSubscribers()) {
            msg_label_color->header.stamp = t_goal->image.header.stamp;
            pub_label_color.publish(msg_label_color);
        }
        if (pub_label.getNumSubscribers()) {
            msg_label->header.stamp = t_goal->image.header.stamp;
            pub_label.publish(msg_label);
        }
    }

    ~SemanticActionServer(void)
    {
        std::cout << "----Server shutdown ----" << std::endl;
        as_.shutdown();
    }
};

cv::Mat generateMask(const cv::Mat& t_label, const cv::Mat t_depth);

void signal_handler_callback(int sig)
{
    ROS_INFO("Ctrl+C calling");

    segnet_instance->stop();
    ros::shutdown();
}

bool SegNet_server_callback(semantic_generator::semanticSegment::Request& req,
    semantic_generator::semanticSegment::Response& res)
{
    cv_bridge::CvImageConstPtr cv_ptrRGBImage, cv_ptrDepthImage;
    std::cout << "Request id: " << req.id << std::endl;
    // ROS_INFO("request: id=%ld", (long int)req.id);
    try {
        cv_ptrRGBImage = cv_bridge::toCvCopy(req.rgb);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    try {
        cv_ptrDepthImage = cv_bridge::toCvCopy(req.depth);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    cv::Mat depth = cv_ptrDepthImage->image;

    cv::Mat label_color, label;
    //thread is not used
    segnet_instance->segNetSegment(cv_ptrRGBImage->image, label, label_color);

    if (label.empty() || label_color.empty()) {
        return false;
    }

    res.id = req.id;
    sensor_msgs::ImagePtr msg_label = cv_bridge::CvImage(std_msgs::Header(), "mono8", label).toImageMsg();

    // cv2RosMessage(label, msg_label);
    res.label = *msg_label;

    sensor_msgs::ImagePtr msg_label_color = cv_bridge::CvImage(std_msgs::Header(), "bgr8", label_color).toImageMsg();

    // cv2RosMessage(label, msg_label_color);
    res.label_color = *msg_label_color;

    cv::Mat mask = generateMask(label, depth); //depth is not used temply
    sensor_msgs::ImagePtr msg_mask = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
    res.mask = *msg_mask;

    if (pub_label_color.getNumSubscribers()) {
        msg_label_color->header.stamp = req.rgb.header.stamp;
        pub_label_color.publish(msg_label_color);
    }
    if (pub_label.getNumSubscribers()) {
        msg_label->header.stamp = req.rgb.header.stamp;
        pub_label.publish(msg_label);
    }

    frame_number++;
    std::cout << "Average time: " << segnet_instance->m_segment_time / frame_number << std::endl;

    return true;
}

void rgbImageCallback(const sensor_msgs::ImageConstPtr& t_cpMsg)
{
    cv_bridge::CvImageConstPtr cv_ptrImage;
    try {
        cv_ptrImage = cv_bridge::toCvShare(t_cpMsg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat label_color, label;

    // segnet_instance->segment(cv_ptrImage->image, label, label_color); //use seperate thread

    segnet_instance->segNetSegment(cv_ptrImage->image, label, label_color); //not use thread

    sensor_msgs::ImagePtr msg_label_color = cv_bridge::CvImage(std_msgs::Header(), "bgr8", label_color).toImageMsg();

    // publish color_label
    if (pub_label_color.getNumSubscribers()) {
        // msg_label_color->header.stamp = t_cpMsg->header.stamp;
        pub_label_color.publish(msg_label_color);
    }
}

// void depthImageCallback(const sensor_msgs::ImageConstPtr &t_cpMsg)
// {
// }
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "SegNet_server");
    ros::NodeHandle nh("~");
    // ros::Rate loop_rate(5);

    caffemodel = std::string("/home/yubao/data/catkin_ws/src/semantic-slam/segnet/models/segnet_pascal.caffemodel");
    prototxt = std::string("/home/yubao/data/catkin_ws/src/semantic-slam/segnet/prototxts/segnet_pascal.prototxt");
    lut = std::string("/home/yubao/data/catkin_ws/src/semantic-slam/segnet/LUT/pascal.png");

    std::string rgb_topic_sub, depth_topic_sub;
    std::string label_topic, label_color_topic, mask_topic;
    std::string semantic_action_topic;

    nh.param("caffemodel", caffemodel, std::string("caffemodel"));
    nh.param("prototxt", prototxt, std::string("prototxt"));
    nh.param("lut", lut, std::string("lut.png"));

    std::cout
        << "Caffemodel: " << caffemodel << std::endl;
    std::cout << "prototxt: " << prototxt << std::endl;
    std::cout << "lut: " << lut << std::endl;

    //Input image, usually color image
    nh.param("/image/rgb_image_topic", rgb_topic_sub, std::string("/camera/rgb/image_color"));
    // nh.param("/image/depth_image_topic", depth_topic_sub, std::string("/camera/depth/image"));

    nh.param("/semantic/mode", mode, 0);
    nh.param("/action/camera_reading", semantic_action_topic, std::string("/semantic_label"));
    nh.param("/semantic/semantic_label", label_topic, std::string("/semantic_label"));
    nh.param("/semantic/semantic_label_color", label_color_topic, std::string("/semantic_label_color"));
    // nh.param("mask_topic", mask_topic, std::string("/semantic_mask"));

    segnet_instance = SegNet::getInstance();
    server = new SemanticActionServer(nh, semantic_action_topic);
    segnet_instance->segnetInit(prototxt, caffemodel, lut);
    if (mode == 0) {
        std::cout << "Use Action Server Mode" << std::endl;
        segnet_instance->start();
    } else if (mode == 1) {
        std::cout << "Use image topic to request semantic label" << std::endl;
        rgb_sub = nh.subscribe(rgb_topic_sub, 10, rgbImageCallback);
    } else if (mode == 3) {
        std::cout << "Use Service Mode" << std::endl;
        service = new ros::ServiceServer(nh.advertiseService("/semantic_generator", SegNet_server_callback));
    }
    // depth_sub = nh.subscribe(depth_topic_sub, 10, depthImageCallback);

    //publish semantic label
    image_transport::ImageTransport it(nh);
    pub_label_color = it.advertise(label_color_topic, 10);
    pub_label = it.advertise(label_topic, 10);
    // pub_color = it.advertise(color_image_topic, 10);
    // pub_depth = it.advertise(depth_image_topic, 10);
    // pub_mask = it.advertise(mask_topic, 10);

    signal(SIGINT, signal_handler_callback);
    // pub_label_color_service = it.advertise("/label_color_service", 10);
    ROS_INFO("SegNet Server Ready ...");
    while (ros::ok()) {
        ros::spinOnce();
        // loop_rate.sleep();
    }

    std::cout << "Server down" << std::endl;
    segnet_instance->stop();
    ros::shutdown();
    return 0;
}

cv::Mat generateMask(const cv::Mat& t_label, const cv::Mat t_depth)
{
    cv::Mat label_mask = (t_label == 15); //15 is person in Pascal VOC
    cv::Mat dialation, close;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
    cv::morphologyEx(label_mask, close, CV_MOP_CLOSE, kernel);
    cv::morphologyEx(close, dialation, CV_MOP_DILATE, kernel);
    label_mask = dialation;
    std::vector<vector<cv::Point>> contours;
    cv::findContours(label_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // std::vector<cv::Point>> contours_poly(contours.size());
    // std::vector<cv::Rect> boundRect(contours.size());
    // std::vector<cv::Point2f> centers(contours.size());
    // std::vector<float> radius(contours.size());

    std::vector<std::vector<cv::Point>> hull(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        // cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        // boundRect[i] = cv::boundingRect(contours_poly[i]);
        cv::convexHull(contours[i], hull[i]);
        cv::fillConvexPoly(label_mask, hull[i], 255);
    }
    cv::morphologyEx(label_mask, close, CV_MOP_CLOSE, kernel);
    label_mask = close;
    return label_mask; //mask
}
