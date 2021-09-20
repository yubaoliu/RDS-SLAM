/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#include "action_server.h"

#include <segnet_ros/segnetAction.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <actionlib/client/simple_action_client.h>

using namespace std;
using namespace segnet_ros;

typedef actionlib::SimpleActionClient<segnet_ros::segnetAction> Client;

class ClientNode {
public:
    ClientNode(ros::NodeHandle& t_nh, const std::string t_name, const bool t_spin)
        : ac_(t_nh, t_name, t_spin)
    {
        ROS_INFO("Waiting for action server to start.");
        std::cout << "Waiting for action server to start." << std::endl;
        ac_.waitForServer();
        ROS_INFO("Action server started, sending goal.");
        factory_id_ = -1;
    }

    void doStuff(const sensor_msgs::ImageConstPtr imgMsg)
    {
        segnetGoal goal;
        goal.id = ++factory_id_;
        goal.image = *imgMsg;

        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&ClientNode::doneCb, this, _1, _2),
            Client::SimpleActiveCallback(),
            Client::SimpleFeedbackCallback());

        std::cout << "Request ID: " << goal.id << std::endl;
        LOG(INFO) << "Request ID: " << goal.id;

        //wait for the action to return
        bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac_.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        } else {
            ROS_INFO("Action did not finish before the time out.");
        }
    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
        const segnetResultConstPtr& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Answer: %i", result->id);

        cv_bridge::CvImageConstPtr cvpLabel, cvpLabelColor;
        try {
            cvpLabel = cv_bridge::toCvCopy(result->label);
            cvpLabelColor = cv_bridge::toCvCopy(result->label_color);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        std::string savePath = "/root/Dataset/results/segnet_result/" + std::to_string(result->id)+".png";
        cv::imwrite(savePath, cvpLabelColor->image);

        cv::imshow("label", cvpLabelColor->image);
        cv::waitKey(1);
    }

private:
    Client ac_;
    long int factory_id_;
};

void signal_handler_callback(int sig)
{
    ROS_INFO("Ctrl+C calling");

    ros::shutdown();
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "SegNet_Client");
    ros::NodeHandle nh("~");
    // ros::Rate loop_rate(5);

    signal(SIGINT, signal_handler_callback);

    // action config
    std::string semantic_action_topic;
    nh.param("/segnet/action_name", semantic_action_topic, std::string("/segnet_action_server"));
    std::cout << "Action name" << semantic_action_topic << std::endl;
    LOG(INFO) << "Action name" << semantic_action_topic;

    std::string bagfile_name, image_topic_name;
    nh.getParam("bagfile_name", bagfile_name);
    nh.getParam("/camera/rgb", image_topic_name);

    std::cout << "bagfile_name: " << bagfile_name << std::endl;
    std::cout << "image_topic_name: " << image_topic_name << std::endl;

    std::cout << "Connecting to action server" << std::endl;
    ClientNode client(nh, semantic_action_topic, true);

    cv::Mat image;
    sensor_msgs::ImagePtr imgMsgPtr;

    rosbag::Bag bag;
    bag.open(bagfile_name);

    ros::Time timer;

    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
        std::string imgTopic = m.getTopic();
        if (image_topic_name == imgTopic) {
            std::cout << "topic name: " << imgTopic << std::endl;
            try {
                imgMsgPtr = m.instantiate<sensor_msgs::Image>();
                image = cv_bridge::toCvCopy(imgMsgPtr)->image;
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Image convert error");
            }

            LOG(INFO) << "------------------------------------------------";
            std::cout << "------------------------------------------------" << std::endl;

            timer = ros::Time::now();
            client.doStuff(imgMsgPtr);
            double elapsedTime = (ros::Time::now() - timer).toSec() * 1000;
            std::cout << "Total time cost of each frame:" << elapsedTime << "ms" << std::endl;
            LOG(INFO) << "Total time cost of each frame:" << elapsedTime << "ms";

            // cv::imshow("image", image);
            // cv::waitKey(1);
        }
    } //for
    bag.close();

    ros::spin();
    // while (ros::ok()) {
    // ros::spinOnce();
    // loop_rate.sleep();
    // }

    std::cout << "Server down" << std::endl;
    ros::shutdown();
    return 0;
}

// Use bag topic
// void rgbImageCallback(const sensor_msgs::ImageConstPtr& t_cpMsg)
// {
//     cv_bridge::CvImageConstPtr cv_ptrImage;
//     try {
//         cv_ptrImage = cv_bridge::toCvShare(t_cpMsg);
//     } catch (cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
//
//     cv::Mat label_color, label;
//
//     // segnet_instance->segment(cv_ptrImage->image, label, label_color); //use seperate thread
//
//     segnet_instance->segNetSegment(cv_ptrImage->image, label, label_color); //not use thread
//
//     sensor_msgs::ImagePtr msg_label_color = cv_bridge::CvImage(std_msgs::Header(), "bgr8", label_color).toImageMsg();
//
//     // publish color_label
//     if (pub_label_color.getNumSubscribers()) {
//         // msg_label_color->header.stamp = t_cpMsg->header.stamp;
//         pub_label_color.publish(msg_label_color);
//     }
// }
