/* 
 * Copyright (C) 2021, Yubao Liu, AISL, TOYOHASHI UNIVERSITY of TECHNOLOGY 
 * Email: yubao.liu.ra@tut.jp 
 * 
 */

#include "Semantic.h"
#include "System.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>

using namespace std;
using namespace ORB_SLAM3;

void spin_thread()
{
    ros::spinOnce();
    usleep(1);
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "tum_rgbd_raw");
    ros::start();
    ros::NodeHandle nh("~");

    if (argc < 5) {
        cerr << endl
             << "Usage: EXE ./Dynamic_RGBD path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        ros::shutdown();
        return 1;
    }

    std::string rgb_image_topic, depth_image_topic, bagfile_name;

    std::cout << "===========================" << std::endl;
    std::cout << "argv[1]: " << argv[1] << std::endl;
    std::cout << "argv[2]: " << argv[2] << std::endl;
    std::cout << "argv[3]: " << argv[3] << std::endl;
    std::cout << "argv[4]: " << argv[4] << std::endl;
    std::cout << "argv[5] result path: " << argv[5] << std::endl;

    LOG(INFO) << "---------Parameters---------------";
    LOG(INFO) << "argv[1]: " << argv[1];
    LOG(INFO) << "argv[2]: " << argv[2];
    LOG(INFO) << "argv[3]: " << argv[3];
    LOG(INFO) << "argv[4]: " << argv[4];
    LOG(INFO) << "argv[5] result path: " << argv[5];
    LOG(INFO) << "----------------------------------";
    std::cout << "===========================" << std::endl;

    // save result
    if (argc == 6) {
        Config::GetInstance()->IsSaveResult(true);
        Config::GetInstance()->createSavePath(std::string(argv[5]));
    } else {
        Config::GetInstance()->IsSaveResult(false);
    }

    std::thread(spin_thread);

    // Retrieve paths to images
    std::vector<std::string> vstrImageFilenamesRGB;
    std::vector<std::string> vstrImageFilenamesD;
    std::vector<double> vTimestamps;
    std::string strAssociationFilename = std::string(argv[4]);

    Config::GetInstance()->LoadTUMDataset(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if (vstrImageFilenamesRGB.empty()) {
        cerr << endl
             << "No images found in provided path." << endl;
        return 1;
    } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
        cerr << endl
             << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    std::string cnn_method = "segnet"; // maskrcnn
    // SegNet case do not need set the "init_delay" and "frame_delay"
    // delay for the initial few frames
    float init_delay = 0.05; //usec
    int init_frames = 2;    //usec
    // delay for each frame
    float frame_delay = 0;

    nh.getParam("cnn_method", cnn_method);
    // control the framerate
    nh.getParam("init_delay", init_delay);
    nh.getParam("frame_delay", frame_delay);
    nh.getParam("init_frames", init_frames);

    LOG(INFO) << "Delay for the initial few frames: " << init_delay;

    Semantic::GetInstance()->SetSemanticMethod(cnn_method);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    Semantic::GetInstance()->Run();

    std::vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cv::Mat imRGB, imD;
    for (int ni = 0; ni < nImages; ni++) {
        // Read image and depthmap from file
        imRGB = cv::imread(std::string(argv[3]) + "/" + vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
        imD = cv::imread(std::string(argv[3]) + "/" + vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGB.empty()) {
            cerr << endl
                 << "Failed to load image at: "
                 << std::string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imD, tframe);

        // Manually add delay to evaluate TUM, because TUM dataset is very short
        if (ni < init_frames) {
            usleep(init_delay);
        } else {
            usleep(frame_delay);
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    LOG(INFO) << "===============Tracking Finished============";
    std::cout << "===============Tracking Finished============" << std::endl;

    // ros::spin();

    LOG(INFO) << "===============Final Stage============";
    std::cout << "===============Final Stage============" << std::endl;

    // Stop semantic thread
    // Semantic::GetInstance()->RequestFinish();

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    std::sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    LOG(INFO) << "median tracking time: " << vTimesTrack[nImages / 2] * 1000 << "ms";
    cout << "mean tracking time: " << totaltime / nImages << endl;
    LOG(INFO) << "mean tracking time: " << totaltime / nImages * 1000 << "ms";

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}
