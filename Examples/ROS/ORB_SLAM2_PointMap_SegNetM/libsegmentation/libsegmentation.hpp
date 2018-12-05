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

using namespace caffe;  

class Classifier
{

    public:
    Classifier(const string& model_file,
    const string& trained_file);

    cv::Mat Predict(const cv::Mat& img,  cv::Mat LUT_image);

    private:
    void SetMean(const string& mean_file);

    void WrapInputLayer(std::vector<cv::Mat>* input_channels);

    void Preprocess(const cv::Mat& img,
    std::vector<cv::Mat>* input_channels);

    private:
    boost::shared_ptr<Net<float> > net_;
    cv::Size input_geometry_;
    cv::Size output_geometry_;
    int num_channels_;

};

#endif
