/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */
#include "maskrcnn_client.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace maskrcnn_ros;

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "maskrcnn_action_client");
    ros::NodeHandle nh("~");

    std::string bagfile_name, image_topic_name;
    nh.getParam("bagfile_name", bagfile_name);
    nh.getParam("/camera/rgb", image_topic_name);

    std::cout << "bagfile_name: " << bagfile_name << std::endl;
    std::cout << "image_topic_name: " << image_topic_name << std::endl;

    std::cout << "Connecting to action server" << std::endl;
    MaskRCNN_Client_Batch my_node(std::string("/maskrcnn_action_server"), true);
    std::cout << "Connected to action server" << std::endl;

    // sensor_msgs::ImageConstPtr imgMsgPtr;

    // Read all the images
    std::vector<cv::Mat> vImages;
    cv::Mat image;
    sensor_msgs::ImagePtr imgMsgPtr;
    rosbag::Bag bag;
    bag.open(bagfile_name);
    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
        std::string imgTopic = m.getTopic();
        if (image_topic_name == imgTopic) {
            // std::cout << "topic name: " << imgTopic << std::endl;
            try {
                imgMsgPtr = m.instantiate<sensor_msgs::Image>();
                image = cv_bridge::toCvCopy(imgMsgPtr)->image;
                vImages.push_back(image);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Image convert error");
            }
        }
    }
    bag.close();

    std::cout << "Read images: " << vImages.size() << std::endl;

    // Request segmentation
    int batch_size = 10;

    std::vector<cv::Mat> vBatchImage;
    for (size_t i = 0; i < vImages.size(); i++) {
        cv::Mat img = vImages[i];
        if ((i != 0) && i % batch_size == 0) {
            if (vBatchImage.size() > 0) {
                std::cout << "Request batch: " << i / batch_size << std::endl;
                std::cout << "Size of requst: " << vBatchImage.size() << std::endl;
                my_node.RequestSegmentation(vBatchImage);
                vBatchImage.clear();

                vBatchImage.push_back(img);
            }
        } else {
            vBatchImage.push_back(img);
        }
    }

    // cv::imshow("image", image);
    // cv::waitKey(1);

    ros::spin();
    return 0;
}
