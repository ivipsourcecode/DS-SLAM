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

#include <iostream>

#include "Camera.h"

using namespace std;

namespace ORB_SLAM2 
{

std::string Camera::name;
float Camera::width;
float Camera::height;
float Camera::fx;
float Camera::fy;
float Camera::cx;
float Camera::cy;
float Camera::invfx;
float Camera::invfy;
float Camera::bf;
float Camera::b;
cv::Mat Camera::K;
cv::Mat Camera::DistCoef;
bool Camera::initialized = false;

bool Camera::Load(const std::string strSettingPath) 
{
    cerr << endl << "Loading camera calibration from: " << strSettingPath << endl;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    Camera::Load(fSettings);
    fSettings.release();
    return true;
}

bool Camera::Load(cv::FileStorage fSettings) 
{
	Camera::width = fSettings["Camera.width"];
	Camera::height = fSettings["Camera.height"];
	Camera::fx = fSettings["Camera.fx"];
	Camera::fy = fSettings["Camera.fy"];
	Camera::cx = fSettings["Camera.cx"];
	Camera::cy = fSettings["Camera.cy"];

	cv::Mat K = cv::Mat::eye(3,3,CV_32F);
	K.at<float>(0,0) = fx;
	K.at<float>(1,1) = fy;
	K.at<float>(0,2) = cx;
	K.at<float>(1,2) = cy;
	K.copyTo(Camera::K);

	cv::Mat DistCoef(4,1,CV_32F);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	if(k3!=0)
	{
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
	}
	DistCoef.copyTo(Camera::DistCoef);

	Camera::bf = fSettings["Camera.bf"];

	Camera::invfx = 1.0f/Camera::fx;
	Camera::invfy = 1.0f/Camera::fy;
	Camera::b = Camera::bf/Camera::fx;

	Camera::initialized = true;
	cout << "- size: " << Camera::width << "x" <<  Camera::height << endl;
	cout << "- fx: " << Camera::fx << endl;
	cout << "- fy: " << Camera::fy << endl;
	cout << "- cx: " << Camera::cx << endl;
	cout << "- cy: " << Camera::cy << endl;
	cout << "- k1: " << Camera::DistCoef.at<float>(0) << endl;
	cout << "- k2: " << Camera::DistCoef.at<float>(1) << endl;
	if(Camera::DistCoef.rows==5)
        cout << "- k3: " << Camera::DistCoef.at<float>(4) << endl;
	cout << "- p1: " << Camera::DistCoef.at<float>(2) << endl;
	cout << "- p2: " << Camera::DistCoef.at<float>(3) << endl;
	cout << "- bf: " << Camera::bf << endl;
	return true;
}

}



