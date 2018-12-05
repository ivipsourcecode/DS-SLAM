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


#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "Camera.h"
#include <Segment.h>
#include <thread>
// The previous image
cv::Mat imGrayPre;
std::vector<cv::Point2f> prepoint, nextpoint;
std::vector<cv::Point2f> F_prepoint, F_nextpoint;
std::vector<cv::Point2f> F2_prepoint, F2_nextpoint;

std::vector<uchar> state;
std::vector<float> err;
std::vector<std::vector<cv::KeyPoint>> mvKeysPre;

namespace ORB_SLAM2
{
long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;
inline void Frame::InitializeScaleLevels() 
{
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
}
  
// Computes image bounds for the undistorted image (called in the constructor)
void ComputeImageBounds()
{
    if(Camera::DistCoef.at<float>(0)!=0.0)
    {
        // Correction boundary point:(0,0),(cols,0),(0,rows),(cols,rows)
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0;
        mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=Camera::width;
        mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0;
        mat.at<float>(2,1)=Camera::height;
        mat.at<float>(3,0)=Camera::width;
        mat.at<float>(3,1)=Camera::height;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat, mat, Camera::K, Camera::DistCoef, cv::Mat(), Camera::K);
        mat=mat.reshape(1);

        Frame::mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        Frame::mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        Frame::mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        Frame::mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
    }
    else
    {
        Frame::mnMinX = 0.0f;
        Frame::mnMaxX = Camera::width;
        Frame::mnMinY = 0.0f;
        Frame::mnMaxY = Camera::height;
    }
}

void Frame::InitializeClass()
{
    if(Frame::mbInitialComputations)
    {
        cerr << "in Frame::InitializeClass" << endl;
        ComputeImageBounds();
        Frame::mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(Frame::mnMaxX-Frame::mnMinX);
        Frame::mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(Frame::mnMaxY-Frame::mnMinY);
        Frame::mbInitialComputations=false;
    }
}

Frame::Frame()
{}
//Copy Constructor
Frame::Frame(const Frame &frame)
   :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
    mTimeStamp(frame.mTimeStamp),
    mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
    mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
    mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
    mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
    mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
    mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
    mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
    mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
    mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

// Constructor for stereo cameras.
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    InitializeScaleLevels();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORBKeyPoints,this,0,imLeft);
    thread threadRight(&Frame::ExtractORBKeyPoints,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    if(mvKeys.empty())
        return;

    N = mvKeys.size();
    UndistortKeyPoints();
    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    InitializeClass();
    AssignFeaturesToGrid();
}

// Constructor for RGB-D cameras for DS_SLAM
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    InitializeScaleLevels();

    // ORB extraction
    std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
    ExtractORBKeyPoints(0,imGray);
    std::chrono::steady_clock::time_point t22 = std::chrono::steady_clock::now();
    orbExtractTime= std::chrono::duration_cast<std::chrono::duration<double> >(t22 - t11).count();

    cv::Mat  imGrayT = imGray;
    // Calculate the dynamic abnormal points and output the T matrix
    if(imGrayPre.data)
    {
        std::chrono::steady_clock::time_point tm1 = std::chrono::steady_clock::now();
        ProcessMovingObject(imGray);
        std::chrono::steady_clock::time_point tm2 = std::chrono::steady_clock::now();
        movingDetectTime= std::chrono::duration_cast<std::chrono::duration<double> >(tm2 - tm1).count();
        std::swap(imGrayPre, imGrayT);
    }
    else
    {
        std::swap(imGrayPre, imGrayT);
        flag_mov=0;
    }

}

void Frame::CalculEverything( cv::Mat &imRGB, const cv::Mat &imGray,const cv::Mat &imDepth,const cv::Mat &imS)
{ 

    int flagprocess = 0;
    for ( int m=0; m<imS.rows; m+=1 )
    {
        for ( int n=0; n<imS.cols; n+=1 )
        {
            int labelnum = (int)imS.ptr<uchar>(m)[n];
            if(labelnum == PEOPLE_LABLE)
            {
                flagprocess=1;
                break;
            }
        }

        if(flagprocess == 1)
        break;
    }

    if(!T_M.empty() && flagprocess )
    {
        std::chrono::steady_clock::time_point tc1 = std::chrono::steady_clock::now();
        flag_mov = mpORBextractorLeft->CheckMovingKeyPoints(imGray,imS,mvKeysTemp,T_M);
        std::chrono::steady_clock::time_point tc2 = std::chrono::steady_clock::now();
        double tc= std::chrono::duration_cast<std::chrono::duration<double> >(tc2 - tc1).count();
        cout << "check time =" << tc*1000 <<  endl;
    }

    ExtractORBDesp(0,imGray);
    N = mvKeys.size();
    if(mvKeys.empty())
    return;
    UndistortKeyPoints();
    ComputeStereoFromRGBD(imDepth);
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    InitializeClass();
    AssignFeaturesToGrid();
}

// Constructor for Monocular cameras.
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
	InitializeScaleLevels();

    // ORB extraction
    ExtractORBKeyPoints(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    InitializeClass();

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];
        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORBKeyPoints(int flag,const cv::Mat &imgray)
{
    if(flag==0)
    {
        (*mpORBextractorLeft)( imgray,cv::Mat(),mvKeysTemp);
    }
    else
        (*mpORBextractorRight)(imgray,cv::Mat(),mvKeysTemp);
}

void Frame::ExtractORBDesp(int flag,const cv::Mat &imgray)
{
    if(flag==0)
        (*mpORBextractorLeft).ProcessDesp(imgray,cv::Mat(),mvKeysTemp,mvKeys,mDescriptors);
    else
        (*mpORBextractorLeft).ProcessDesp(imgray,cv::Mat(),mvKeysTemp,mvKeysRight,mDescriptorsRight);
    
}

// Epipolar constraints and output the T matrix.
void Frame::ProcessMovingObject(const cv::Mat &imgray)
{
    // Clear the previous data
	F_prepoint.clear();
	F_nextpoint.clear();
	F2_prepoint.clear();
	F2_nextpoint.clear();
	T_M.clear();

	// Detect dynamic target and ultimately optput the T matrix
	
    cv::goodFeaturesToTrack(imGrayPre, prepoint, 1000, 0.01, 8, cv::Mat(), 3, true, 0.04);
    cv::cornerSubPix(imGrayPre, prepoint, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
	cv::calcOpticalFlowPyrLK(imGrayPre, imgray, prepoint, nextpoint, state, err, cv::Size(22, 22), 5, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));

	for (int i = 0; i < state.size(); i++)
    {
        if(state[i] != 0)
        {
            int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
            int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
            int x1 = prepoint[i].x, y1 = prepoint[i].y;
            int x2 = nextpoint[i].x, y2 = nextpoint[i].y;
            if ((x1 < limit_edge_corner || x1 >= imgray.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= imgray.cols - limit_edge_corner
            || y1 < limit_edge_corner || y1 >= imgray.rows - limit_edge_corner || y2 < limit_edge_corner || y2 >= imgray.rows - limit_edge_corner))
            {
                state[i] = 0;
                continue;
            }
            double sum_check = 0;
            for (int j = 0; j < 9; j++)
                sum_check += abs(imGrayPre.at<uchar>(y1 + dy[j], x1 + dx[j]) - imgray.at<uchar>(y2 + dy[j], x2 + dx[j]));
            if (sum_check > limit_of_check) state[i] = 0;
            if (state[i])
            {
                F_prepoint.push_back(prepoint[i]);
                F_nextpoint.push_back(nextpoint[i]);
            }
        }
    }
    // F-Matrix
    cv::Mat mask = cv::Mat(cv::Size(1, 300), CV_8UC1);
    cv::Mat F = cv::findFundamentalMat(F_prepoint, F_nextpoint, mask, cv::FM_RANSAC, 0.1, 0.99);
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<uchar>(i, 0) == 0);
        else
        {
            // Circle(pre_frame, F_prepoint[i], 6, Scalar(255, 255, 0), 3);
            double A = F.at<double>(0, 0)*F_prepoint[i].x + F.at<double>(0, 1)*F_prepoint[i].y + F.at<double>(0, 2);
            double B = F.at<double>(1, 0)*F_prepoint[i].x + F.at<double>(1, 1)*F_prepoint[i].y + F.at<double>(1, 2);
            double C = F.at<double>(2, 0)*F_prepoint[i].x + F.at<double>(2, 1)*F_prepoint[i].y + F.at<double>(2, 2);
            double dd = fabs(A*F_nextpoint[i].x + B*F_nextpoint[i].y + C) / sqrt(A*A + B*B); //Epipolar constraints
            if (dd <= 0.1)
            {
                F2_prepoint.push_back(F_prepoint[i]);
                F2_nextpoint.push_back(F_nextpoint[i]);
            }
        }
    }
    F_prepoint = F2_prepoint;
    F_nextpoint = F2_nextpoint;

    for (int i = 0; i < prepoint.size(); i++)
    {
        if (state[i] != 0)
        {
            double A = F.at<double>(0, 0)*prepoint[i].x + F.at<double>(0, 1)*prepoint[i].y + F.at<double>(0, 2);
            double B = F.at<double>(1, 0)*prepoint[i].x + F.at<double>(1, 1)*prepoint[i].y + F.at<double>(1, 2);
            double C = F.at<double>(2, 0)*prepoint[i].x + F.at<double>(2, 1)*prepoint[i].y + F.at<double>(2, 2);
            double dd = fabs(A*nextpoint[i].x + B*nextpoint[i].y + C) / sqrt(A*A + B*B);

            // Judge outliers
            if (dd <= limit_dis_epi) continue;
            T_M.push_back(nextpoint[i]);
        }
    }

}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY = Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=Camera::fx*PcX*invz+Camera::cx;
    const float v=Camera::fy*PcY*invz+Camera::cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,mfLogScaleFactor);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - Camera::bf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    // Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
	    // Transform a set of descriptors into a bow vector and a feature vector
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
  if(Camera::DistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat, mat, Camera::K, Camera::DistCoef, cv::Mat(), Camera::K);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}


void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    // Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = Camera::b;
    const float minD = -3;
    const float maxD = Camera::bf/minZ;  

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<ORBmatcher::TH_HIGH)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=0 && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=Camera::bf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-Camera::bf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-Camera::cx)*z*Camera::invfx;
        const float y = (v-Camera::cy)*z*Camera::invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
