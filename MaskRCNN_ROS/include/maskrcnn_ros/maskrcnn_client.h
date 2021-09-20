// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <image_transport/image_transport.h>

// maskrcnn
#include <maskrcnn_ros/batchAction.h> // Note: "Action" is appended

#include <ros/ros.h>

//log
#include <glog/logging.h>

namespace maskrcnn_ros {
typedef actionlib::SimpleActionClient<maskrcnn_ros::batchAction> Client;

class MaskRCNN_Client_Batch {
public:
    typedef std::shared_ptr<MaskRCNN_Client_Batch> Ptr;
    batchResultConstPtr mResult;

    MaskRCNN_Client_Batch(const std::string& t_name, const bool t_spin)
        : ac(t_name, t_spin)
    {
        LOG(INFO) << "Waiting for action server to start.";
        std::cout << "Waiting for action server to start." << std::endl;
        ac.waitForServer();
        LOG(INFO) << "Action server ready";
        factory_id_ = -1;
    }

    // void Segment(const std::vector<cv::Mat>& in_images, std::vector<cv::Mat>& out_label, std::vector<cv::Mat>& out_score, int out_object_num)
    void Segment(const std::vector<cv::Mat>& in_images, std::vector<cv::Mat>& out_label)
    {
        std::cout << "-----------------" << std::endl;
        int batch_size = in_images.size();
        if (batch_size <= 0) {
            LOG(ERROR) << "No image data";
            return;
        }

        batchGoal goal;
        goal.id = ++factory_id_;

        for (size_t i = 0; i < batch_size; i++) {
            cv_bridge::CvImage cvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, in_images[i]);
            // smImages[i] = *(cvImage.toImageMsg());
            goal.image.push_back(*(cvImage.toImageMsg()));
        }

        // goal.image = smImages;

        std::cout << "Sending request: " << goal.id << std::endl;
        LOG(INFO) << "Sending request: " << goal.id;
        // std::cout << "batch size: " << goal.batch_size << std::endl;

        // Need boost::bind to pass in the 'this' pointer
        ac.sendGoal(goal);

        std::cout << "Request ID: " << goal.id << std::endl;

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        } else {
            ROS_INFO("Action did not finish before the time out.");
        }
        mResult = ac.getResult();
        // out_object_num = mResult->object_num;
        for (size_t i = 0; i < batch_size; i++) {
            cv::Mat label = cv_bridge::toCvCopy(mResult->label[i])->image;
            // cv::Mat score = cv_bridge::toCvCopy(mResult->score[i])->image;
            out_label.push_back(label);
            // out_score.push_back(score);
        }
    }

    void RequestSegmentation(const std::vector<cv::Mat>& inBatchImages)
    {
        std::cout << "-----------------" << std::endl;
        int batch_size = inBatchImages.size();
        if (batch_size <= 0) {
            LOG(ERROR) << "No image data";
            std::cout << "No image data" << std::endl;
            return;
        }

        batchGoal goal;
        goal.id = ++factory_id_;
        // goal.batch_size = batch_size;

        for (size_t i = 0; i < batch_size; i++) {
            cv_bridge::CvImage cvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, inBatchImages[i]);
            // smImages[i] = *(cvImage.toImageMsg());
            goal.image.push_back(*(cvImage.toImageMsg()));
        }

        // goal.image = smImages;

        std::cout << "Sending request: " << goal.id << std::endl;
        // std::cout << "batch size: " << goal.batch_size << std::endl;

        // Need boost::bind to pass in the 'this' pointer
        ac.sendGoal(goal,
            boost::bind(&MaskRCNN_Client_Batch::doneCb, this, _1, _2),
            Client::SimpleActiveCallback(),
            Client::SimpleFeedbackCallback());

        std::cout << "Request ID: " << goal.id << std::endl;

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        } else {
            ROS_INFO("Action did not finish before the time out.");
        }
    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
        const batchResultConstPtr& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Answer: %i", result->id);
        // ros::shutdown();
    }

private:
    Client ac;
    long int factory_id_;
};
}
