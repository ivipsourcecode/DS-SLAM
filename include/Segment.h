/*
 *--------------------------------------------------------------------------------------------------
 * DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments
　*　Author(s):
 * Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei Qiao qiaofei@mail.tsinghua.edu.cn
 * Created by Yu Chao@2018.12.03
 * --------------------------------------------------------------------------------------------------
 * DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2. If you haven't learn ORB_SLAM2 code, 
 * you'd better to be familiar with ORB_SLAM2 project first. Compared to ORB_SLAM2, 
 * we add anther two threads including semantic segmentation thread and densemap creation thread. 
 * You should pay attention to Frame.cc, ORBmatcher.cc, Pointcloudmapping.cc and Segment.cc.
 * 
 *　@article{murORB2,
 *　title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
　*　author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
　* journal={IEEE Transactions on Robotics},
　*　volume={33},
　* number={5},
　* pages={1255--1262},
　* doi = {10.1109/TRO.2017.2705103},
　* year={2017}
 *　}
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2018, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/) and 
 * Advanced Mechanism and Roboticized Equipment Lab. All rights reserved.
 *
 * Licensed under the GPLv3 License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://github.com/ivipsourcecode/DS-SLAM/blob/master/LICENSE
 *--------------------------------------------------------------------------------------------------
 */

#ifndef SEGMENT_H
#define SEGMENT_H

#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "libsegmentation.hpp" 
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>

namespace ORB_SLAM2
{

class Tracking;

class Segment
{

public:
    Segment(const string &pascal_prototxt, const string &pascal_caffemodel, const string &pascal_png);
    void SetTracker(Tracking* pTracker);
    void Run();
    int conbase = 64, jinzhi=4;
    int labeldata[20]={32,8,40,2,34,10,42,16,48,24,56,18,50,26,58,4,36,12,44,6};

    cv::Mat label_colours;
    Classifier* classifier;
    bool isNewImgArrived();
    bool CheckFinish();
    void RequestFinish();
    void Initialize(const cv::Mat& img);
    cv::Mat mImg;
    cv::Mat mImgTemp;
    cv::Mat mImgSegment_color;
    cv::Mat mImgSegment_color_final;
    cv::Mat mImgSegment;
    cv::Mat mImgSegmentLatest;
    Tracking* mpTracker;
    std::mutex mMutexGetNewImg;
    std::mutex mMutexFinish;
    bool mbFinishRequested;
    void ProduceImgSegment();
    std::mutex mMutexNewImgSegment;
    std::condition_variable mbcvNewImgSegment;
    bool mbNewImgFlag;
    int mSkipIndex;
    double mSegmentTime;
    int imgIndex;
    // Paremeters for caffe
    string model_file;
    string trained_file;
    string LUT_file;
};

}



#endif
