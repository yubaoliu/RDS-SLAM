/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#ifndef _SEGNET_H_
#define _SEGNET_H_

#include "common.h"
#include "libsegnet.h"

namespace segnet_ros {
class SegNet {
public:
    static SegNet* getInstance();
    void segnetInit(const std::string& t_prototxt, const std::string& t_caffemodel, const std::string& t_lut_image);
    void start();
    void stop();
    // segment use multiple thread
    void SegmentMultiThread(const cv::Mat& t_img, cv::Mat& t_label, cv::Mat& t_label_color);
    // segment image without multiple thread
    bool SegmentSingleThread(const cv::Mat& t_img, cv::Mat& t_label, cv::Mat& t_label_color);
    // void cv2RosMessage(const cv::Mat &t_image, sensor_msgs::Image &t_msg_out);
    
    double m_segment_time;

private:
    void SegmentImage(const cv::Mat& t_img);
    static SegNet* m_instance;

    void Run();
    SegNet();
    boost::thread* m_thSegNet;

    boost::mutex m_mutex_bIsSegnetThreadEnabled;
    bool bIsSegnetThreadEnabled;

    boost::mutex m_mutex_produce, m_mutex_consume;
    boost::condition_variable m_cvProduct_produce;
    boost::condition_variable m_cvProduct_consume;

    ~SegNet();
    std::string m_model_file;
    std::string m_trained_file;
    std::string m_lut_file;
    cv::Mat m_label, m_label_color;
    cv::Mat m_lut_imgae;

    boost::mutex m_mutex_image;
    cv::Mat m_image;
    Classifier* m_classifier;
};

} // namespace segnet_ros
#endif
