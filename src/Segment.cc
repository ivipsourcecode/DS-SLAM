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

#include "Segment.h"
#include "Tracking.h"
#include "Camera.h"
#include <fstream>
#define SKIP_NUMBER 1
using namespace std;

namespace ORB_SLAM2
{
Segment::Segment(const string &pascal_prototxt, const string &pascal_caffemodel, const string &pascal_png):mbFinishRequested(false),mSkipIndex(SKIP_NUMBER),mSegmentTime(0),imgIndex(0)
{

    model_file = pascal_prototxt;
    trained_file = pascal_caffemodel;
    LUT_file = pascal_png;
    label_colours = cv::imread(LUT_file,1);
    cv::cvtColor(label_colours, label_colours, CV_RGB2BGR);
    mImgSegmentLatest=cv::Mat(Camera::height,Camera::width,CV_8UC1);
    mbNewImgFlag=false;

}

void Segment::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

bool Segment::isNewImgArrived()
{
    unique_lock<mutex> lock(mMutexGetNewImg);
    if(mbNewImgFlag)
    {
        mbNewImgFlag=false;
        return true;
    }
    else
    return false;
}

void Segment::Run()
{
    classifier=new Classifier(model_file, trained_file);
    cout << "Load model ..."<<endl;
    while(1)
    {

        usleep(1);
        if(!isNewImgArrived())
        continue;

        cout << "Wait for new RGB img time =" << endl;
        if(mSkipIndex==SKIP_NUMBER)
        {
            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
            // Recognise by Semantin segmentation
            mImgSegment=classifier->Predict(mImg, label_colours);

            mImgSegment_color = mImgSegment.clone();
            cv::cvtColor(mImgSegment,mImgSegment_color, CV_GRAY2BGR);

            LUT(mImgSegment_color, label_colours, mImgSegment_color_final);
            cv::resize(mImgSegment, mImgSegment, cv::Size(Camera::width,Camera::height) );
            cv::resize(mImgSegment_color_final, mImgSegment_color_final, cv::Size(Camera::width,Camera::height) );

            std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
            mSegmentTime+=std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count();
            mSkipIndex=0;
            imgIndex++;
        }
        mSkipIndex++;
        ProduceImgSegment();
        if(CheckFinish())
        {
            break;
        }

    }

}

bool Segment::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}
  
void Segment::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested=true;
}

void Segment::ProduceImgSegment()
{
    std::unique_lock <std::mutex> lock(mMutexNewImgSegment);
    mImgTemp=mImgSegmentLatest;
    mImgSegmentLatest=mImgSegment;
    mImgSegment=mImgTemp;
    mpTracker->mbNewSegImgFlag=true;
}

}
