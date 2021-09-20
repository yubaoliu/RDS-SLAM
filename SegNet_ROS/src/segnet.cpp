/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#include "segnet.h"

namespace segnet_ros {

SegNet::SegNet()
{
    m_model_file = std::string("");
    m_trained_file = std::string("");
    m_lut_file = std::string("");
    bIsSegnetThreadEnabled = true;
}

void SegNet::start()
{
    m_thSegNet = new boost::thread(&SegNet::Run, this);
}

void SegNet::stop()
{
    boost::unique_lock<boost::mutex> lock(m_mutex_bIsSegnetThreadEnabled);
    bIsSegnetThreadEnabled = false;
    lock.unlock();
    usleep(5);

    if (m_thSegNet->joinable()) {
        m_thSegNet->join();
    }
}

void SegNet::SegmentMultiThread(const cv::Mat& t_img, cv::Mat& t_label, cv::Mat& t_label_color)
{
    boost::unique_lock<boost::mutex> lock_image(m_mutex_image);
    m_image = t_img;
    // t_img.copyTo(m_image);
    m_cvProduct_produce.notify_one();
    lock_image.unlock();

    boost::unique_lock<boost::mutex> lock_consume(m_mutex_consume);
    m_cvProduct_consume.wait(lock_consume);

    t_label = m_label;
    t_label_color = m_label_color;
    lock_consume.unlock();
}

void SegNet::segnetInit(const string& t_prototxt, const string& t_caffemodel, const string& t_lut_image)
{
    LOG(INFO) << "SegNet Init";

    m_model_file = t_prototxt;
    m_trained_file = t_caffemodel;
    m_lut_file = t_lut_image;

    LOG(INFO) << "model file: " << m_model_file;
    LOG(INFO) << "Trained file: " << m_trained_file;
    LOG(INFO) << "LUT_file: " << m_lut_file;

    m_lut_imgae = cv::imread(m_lut_file, 1);
    if (m_lut_imgae.empty()) {
        LOG(ERROR) << "LUT image is empty";
        return;
    }
    //Author change B and G chanel
    cv::cvtColor(m_lut_imgae, m_lut_imgae, CV_RGB2BGR);
    m_classifier = new Classifier(m_model_file, m_trained_file);
    LOG(INFO) << "Load model ...";
}

void SegNet::SegmentImage(const cv::Mat& t_img)
{
    double predict_time;
    m_label = m_classifier->Predict(t_img, m_lut_imgae, predict_time);
    m_segment_time += predict_time;

    cv::Mat label_color_tmp = m_label.clone();
    cv::cvtColor(m_label, label_color_tmp, CV_GRAY2BGR);

    LUT(label_color_tmp, m_lut_imgae, m_label_color); //LUT:: Look up table
    cv::resize(m_label, m_label, m_image.size());
    cv::resize(m_label_color, m_label_color, m_image.size());
}

// Segmnet Withoud thread
bool SegNet::SegmentSingleThread(const cv::Mat& t_img, cv::Mat& t_label, cv::Mat& t_label_color)
{
    // boost::unique_lock<boost::mutex> lock(m_mutex_produce);
    if (t_img.empty()) {
        return false;
    }

    double predict_time;
    cv::Mat label_tmp = m_classifier->Predict(t_img, m_lut_imgae, predict_time);
    m_segment_time += predict_time;

    cv::Mat label_color_tmp = label_tmp.clone();
    cv::cvtColor(label_tmp, label_color_tmp, CV_GRAY2BGR);
    LUT(label_color_tmp, m_lut_imgae, label_color_tmp); //LUT:: Look up table
    // boost::unique_lock<boost::shared_mutex> lock2(m_mutex_label);
    cv::resize(label_tmp, t_label, t_img.size());
    cv::resize(label_color_tmp, t_label_color, t_img.size());
    // m_segment_generated = true;
    return true;
}

void SegNet::Run()
{
    LOG(INFO) << "Segnet thread starting";
    m_classifier = new Classifier(m_model_file, m_trained_file);
    LOG(INFO) << "Load model ...";

    while (bIsSegnetThreadEnabled) {
        boost::unique_lock<boost::mutex> lock_produce(m_mutex_produce);
        m_cvProduct_produce.wait(lock_produce);
        std::cout << "------------------------------" << std::endl;
        LOG(INFO) << "------------------------------";

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        SegmentImage(this->m_image);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double timer = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        std::cout << "Time cost of per segmentation: " << timer * 1000 << "ms" << std::endl;
        LOG(INFO) << "Time cost of per segmentation: " << timer * 1000 << "ms";

        m_cvProduct_consume.notify_one();
    }
    std::cout << "SegNet thread exit Ok!!" << std::endl;
    LOG(INFO) << "SegNet thread exit Ok!!";
}

// void SegNet::cv2RosMessage(const cv::Mat &t_image, sensor_msgs::ImagePtr &t_msg_out)
// {
//     sensor_msgs::ImagePtr msg;
//     msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", t_image).toImageMsg();
//     t_msg_out = *msg;
// }

// void SegNet::publishLabelColor(const cv::Mat &t_label_color)
// {
// }

SegNet* SegNet::getInstance()
{
    if (m_instance == 0) {
        m_instance = new SegNet();
    }
    LOG(INFO) << "Get SegNet class m_instance";
    return m_instance;
}

SegNet::~SegNet()
{
}

SegNet* SegNet::m_instance = 0;
} // namespace segnet_ros
