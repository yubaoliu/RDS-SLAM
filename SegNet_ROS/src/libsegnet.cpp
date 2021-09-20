#include "libsegnet.h"
#include <chrono>

using namespace caffe;

class Classifier;

Classifier::Classifier(const string& t_model_file,
    const string& t_trained_file)
{
    LOG(INFO) << "Classifier";

#ifdef CPU_ONLY
    Caffe::set_mode(Caffe::CPU);
#else
    Caffe::set_mode(Caffe::GPU);
#endif

    LOG(INFO) << "model_file: " << t_model_file;
    LOG(INFO) << "trained_file: " << t_trained_file;

    net_.reset(new Net<float>(t_model_file, TEST));
    net_->CopyTrainedLayersFrom(t_trained_file);
    Blob<float>* input_layer = net_->input_blobs()[0];
    num_channels_ = input_layer->channels();
    LOG(INFO) << "num_channels_: " << num_channels_;

    input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

    LOG(INFO) << "Input Image width: " << input_layer->width();
    LOG(INFO) << "Input image height: " << input_layer->height();
    LOG(INFO) << "Classifier return";
}

cv::Mat Classifier::Predict(const cv::Mat& img, cv::Mat LUT_image, double& t_predit_time)
{

    Blob<float>* input_layer = net_->input_blobs()[0];

    input_layer->Reshape(1, num_channels_, input_geometry_.height, input_geometry_.width);

    net_->Reshape();

    std::vector<cv::Mat> input_channels;
    WrapInputLayer(&input_channels);
    output_geometry_ = img.size();

    Preprocess(img, &input_channels);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    net_->Forward();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double track = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    std::cout << "process time: " << track * 1000 << "ms" << std::endl;
    // LOG(INFO) << "process time =" << track * 1000;
    t_predit_time = track;

    Blob<float>* output_layer = net_->output_blobs()[0];
    int width = output_layer->width();
    int height = output_layer->height();
    int channels = output_layer->channels();

    cv::Mat class_each_row(channels, width * height, CV_32FC1, const_cast<float*>(output_layer->cpu_data()));
    class_each_row = class_each_row.t();

    cv::Point maxId;
    double maxValue;
    cv::Mat prediction_map(height, width, CV_8UC1);

    for (int i = 0; i < class_each_row.rows; i++) {
        cv::minMaxLoc(class_each_row.row(i), 0, &maxValue, 0, &maxId);
        // LOG(INFO) << "maxValue: " << maxValue;

        if (maxValue > 0.8) {
            prediction_map.at<uchar>(i) = maxId.x;
        } else {
            prediction_map.at<uchar>(i) = 0;
        }
    }
    return prediction_map;
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */

void Classifier::WrapInputLayer(std::vector<cv::Mat>* input_channels)
{

    Blob<float>* input_layer = net_->input_blobs()[0];

    int width = input_layer->width();
    int height = input_layer->height();
    float* input_data = input_layer->mutable_cpu_data();
    for (int i = 0; i < input_layer->channels(); ++i) {
        cv::Mat channel(height, width, CV_32FC1, input_data);
        input_channels->push_back(channel);
        input_data += width * height;
    }
}

void Classifier::Preprocess(const cv::Mat& img,
    std::vector<cv::Mat>* input_channels)
{
    // Convert the input image to the input image format of the network
    cv::Mat sample;

    if (img.channels() == 3 && num_channels_ == 1)
        cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
    else if (img.channels() == 4 && num_channels_ == 1)
        cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
    else if (img.channels() == 4 && num_channels_ == 3)
        cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
    else if (img.channels() == 1 && num_channels_ == 3)
        cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
    else
        sample = img;
    cv::Mat sample_resized;
    if (sample.size() != input_geometry_)
        cv::resize(sample, sample_resized, input_geometry_);
    else
        sample_resized = sample;

    cv::Mat sample_float;
    if (num_channels_ == 3)
        sample_resized.convertTo(sample_float, CV_32FC3);
    else
        sample_resized.convertTo(sample_float, CV_32FC1);
    cv::split(sample_float, *input_channels);
}
