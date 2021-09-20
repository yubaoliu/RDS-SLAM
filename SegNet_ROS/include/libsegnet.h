#ifndef LIBSEGMENTATION_HPP
#define LIBSEGMENTATION_HPP

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <sstream>
//log
#include <glog/logging.h>

using namespace caffe;

class Classifier
{

public:
    Classifier(const string &t_model_file,
               const string &t_trained_file);

    cv::Mat Predict(const cv::Mat &img, cv::Mat LUT_image, double &t_predit_time);

private:
    void SetMean(const string &mean_file);

    void WrapInputLayer(std::vector<cv::Mat> *input_channels);

    void Preprocess(const cv::Mat &img,
                    std::vector<cv::Mat> *input_channels);

private:
    boost::shared_ptr<Net<float>> net_;
    cv::Size input_geometry_;
    cv::Size output_geometry_;
    int num_channels_;
};

#endif
