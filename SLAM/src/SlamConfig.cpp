/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#include "SlamConfig.h"

namespace ORB_SLAM3 {

bool Config::IsSaveResult(const bool t_bIsSaveResult)
{
    mbIsSaveResult = t_bIsSaveResult;
    LOG(INFO) << "IsSaveResult:" << t_bIsSaveResult;
    return mbIsSaveResult;
}

Config* Config::GetInstance()
{
    if (config_ == nullptr) {
        config_ = new Config();
    }
    return config_;
}

Config::Config()
{
}

Config::~Config()
{
}

void Config::createSavePath(const std::string dir)
{
    LOG(INFO) << "Create save path: " << dir;
    config_->root_save_path_ = dir;
    std::string str = dir;
    config_->createDirectory(str); // root directory

    str = dir + "/rgb/";
    config_->createDirectory(str);

    str = dir + "/depth/";
    config_->createDirectory(str);

    str = dir + "/mask/";
    config_->createDirectory(str);

    str = dir + "/label/";
    config_->createDirectory(str);

    str = dir + "/feature/";
    config_->createDirectory(str);

    str = dir + "/label_color/";
    config_->createDirectory(str);

    str = dir + "/debug/";
    config_->createDirectory(str);

    str = dir + "/score/";
    config_->createDirectory(str);
}

void Config::createDirectory(const std::string dir)
{
    boost::filesystem::create_directories(dir);
    // DIR* _dir = opendir(dir.c_str());
    // if (_dir) {
    //     closedir(_dir);
    // } else if (ENOENT == errno) {
    //     const int check = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    //     if (check == -1) {
    //         LOG(WARNING) << "Create directory failed: " << dir;
    //     }
    // }
}

void Config::saveImage(const cv::Mat& image, const std::string relative_dir, const std::string name)
{
    if (!mbIsSaveResult)
        return;
    std::string path = config_->root_save_path_ + "/" + relative_dir + "/" + name;
    if (!image.empty()) {
        cv::imwrite(path, image);
    } else {
        LOG(WARNING) << "no image data: " << path;
    }
}

void Config::LoadTUMDataset(const std::string& strAssociationFilename, std::vector<std::string>& vstrImageFilenamesRGB, std::vector<std::string>& vstrImageFilenamesD, std::vector<double>& vTimestamps)
{
    std::ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while (!fAssociation.eof()) {
        std::string s;
        std::getline(fAssociation, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}

Config* Config::config_ = nullptr;
}
