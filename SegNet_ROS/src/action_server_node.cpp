/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#include "action_server.h"

using namespace std;
using namespace segnet_ros;

void signal_handler_callback(int sig)
{
    ROS_INFO("Ctrl+C calling");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "SegNet_server");
    ros::NodeHandle nh("~");
    // ros::Rate loop_rate(5);

    signal(SIGINT, signal_handler_callback);

    // Segnet config
    std::string caffemodel = std::string("/root/cnnmodel/segnet/segnet_pascal.caffemodel");
    std::string prototxt = std::string("/root/semseg/SegNet/Example_Models/segnet_pascal.prototxt");
    std::string lut = std::string("/root/semseg/LUT/pascal.png");

    nh.param("/segnet/caffemodel", caffemodel, std::string("caffemodel"));
    nh.param("/segnet/prototxt", prototxt, std::string("prototxt"));
    nh.param("/segnet/lut", lut, std::string("lut.png"));

    // action config
    std::string semantic_action_topic;
    nh.param("/segnet/action_name", semantic_action_topic, std::string("/segnet_action_server"));

    // publish result
    std::string label_topic, label_color_topic;
    nh.param("/segnet/label_topic", label_topic, std::string("/segnet/label"));
    nh.param("/segnet/label_color_topic", label_color_topic, std::string("/segnet/label_color"));

    // start action server
    SegnetActionServer* server = new SegnetActionServer(nh, semantic_action_topic, prototxt, caffemodel, lut, label_topic, label_color_topic);

    ROS_INFO("SegNet Server Ready ...");

    ros::spin();
    // while (ros::ok()) {
    // ros::spinOnce();
    // loop_rate.sleep();
    // }
    std::cout << "[SegNet] Server down" << std::endl;
    ros::shutdown();
    return 0;
}
