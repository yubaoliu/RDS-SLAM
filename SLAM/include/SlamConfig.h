/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Common.h"

// Read config file
namespace ORB_SLAM3 {

class Config {
private:
    static Config* config_;
    std::string root_save_path_;
    // private constructor makes a singleton
    Config();
    bool mbIsSaveResult;

public:
    // close the file when deconstructing
    ~Config();

    static Config* GetInstance();

    bool IsSaveResult(const bool t_bIsSaveResult);

    // save result
    void createSavePath(const std::string dir);
    void createDirectory(const std::string dir);
    /*
     * relative_path: refer createdirectory, e.g. rgb, depth,mask ...
     */
    void saveImage(const cv::Mat& image, const std::string relative_dir, const std::string name);

    void LoadTUMDataset(const std::string& strAssociationFilename, std::vector<std::string>& vstrImageFilenamesRGB, std::vector<std::string>& vstrImageFilenamesD, std::vector<double>& vTimestamps);
};
}

#endif
