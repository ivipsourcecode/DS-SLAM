/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

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

#ifndef TRACKING_H
#define TRACKING_H

#include "Frame.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "System.h"
#include "boost/make_shared.hpp"
#include <condition_variable>
#include <mutex>

using std::string;

class PointCloudMapping;

namespace ORB_SLAM2
{

class Viewer;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class Segment;

class Tracking
{ 

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, Map* pMap, boost::shared_ptr<PointCloudMapping> pPointCloud, 
	     KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

public:
    // Tracking states
    enum eTrackingState
    {
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;
    double orbExtractTime;
    double movingDetectTime;
    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;
    cv::Mat mImS;
    cv::Mat mImS_C;
    cv::Mat mImRGB;// adding for color point map  by zoe
    cv::Mat mImDepth; // adding mImDepth member to realize pointcloud view

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();
    // For the semantic segmentation thread
    Segment* mpSegment;
    cv::Mat mImgNew;
    std::condition_variable mbcvImgNew;
    void GetImg(const cv::Mat &img);
    void SetSegment(Segment* segment);
    bool isNewSegmentImgArrived();
    bool mbNewSegImgFlag;

protected:
    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    // Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    // ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    // BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    // Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    // Drawers
    Viewer* mpViewer;

    // Map
    Map* mpMap;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    // Motion Model
    cv::Mat mVelocity;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;
	
	// For point cloud viewing
    boost::shared_ptr<PointCloudMapping> mpPointCloudMapping;

    list<MapPoint*> mlpTemporalPoints;
private:
	bool _Track_full();
	bool _Track_loc_only();
};

} //namespace ORB_SLAM

#endif // TRACKING_H
