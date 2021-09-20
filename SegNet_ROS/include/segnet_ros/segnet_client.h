/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#include <segnet_ros/segnetAction.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <actionlib/client/simple_action_client.h>

using namespace std;
// using namespace segnet_ros;

typedef actionlib::SimpleActionClient<segnet_ros::segnetAction> Client;

namespace segnet_ros {

class SegNetClient {
public:
    typedef std::shared_ptr<SegNetClient> Ptr;
    segnetResultConstPtr mResult;

    SegNetClient(const std::string t_name, const bool t_spin)
        : ac_(t_name, t_spin)
    {
        ROS_INFO("Waiting for action server to start.");
        std::cout << "Waiting for action server to start." << std::endl;
        ac_.waitForServer();
        ROS_INFO("Action server started, sending goal.");
        factory_id_ = -1;
    }

    void Segment(const std::vector<cv::Mat>& in_images, std::vector<cv::Mat>& out_label)
    {
        segnetGoal goal;
        cv_bridge::CvImageConstPtr cvpLabel;
        int batch_size = in_images.size();
        if (batch_size <= 0) {
            LOG(ERROR) << "No image data";
            return;
        }
        for (size_t i = 0; i < batch_size; i++) {
            cv_bridge::CvImage cvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, in_images[i]);
            goal.image = *(cvImage.toImageMsg());
            goal.id = ++factory_id_;

            ac_.sendGoal(goal);

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

            mResult = ac_.getResult();
            cv::Mat label = cv_bridge::toCvCopy(mResult->label)->image;
            out_label.push_back(label);
        } // end for batch
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

        cv::imshow("label", cvpLabelColor->image);
        cv::waitKey(1);
    }

    void doStuff(const sensor_msgs::ImageConstPtr imgMsg)
    {
        segnetGoal goal;
        goal.id = ++factory_id_;
        goal.image = *imgMsg;

        // Need boost::bind to pass in the 'this' pointer
        ac_.sendGoal(goal,
            boost::bind(&SegNetClient::doneCb, this, _1, _2),
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

private:
    Client ac_;
    long int factory_id_;
};
}
