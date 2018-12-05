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

#include<mutex>
#include<climits>
#include <sys/stat.h>

#include "Map.h"
#include "Converter.h"
#include "Camera.h"
#include "ORBextractor.h"
#include "MapPoint.h"
#include "Frame.h"
#include "KeyFrame.h"


namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId > mnMaxKFid)
        mnMaxKFid = pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

KeyFrame* Map::_ReadKeyFrame(ifstream &f, ORBVocabulary &voc, std::vector<MapPoint*> amp, ORBextractor* orb_ext)
{
    Frame fr;
    fr.mpORBvocabulary = &voc;
    f.read((char*)&fr.mnId, sizeof(fr.mnId));
    cerr << " reading keyfrane id " << fr.mnId << endl;
    f.read((char*)&fr.mTimeStamp, sizeof(fr.mTimeStamp));
    cv::Mat Tcw(4,4,CV_32F);
    f.read((char*)&Tcw.at<float>(0, 3), sizeof(float));
    f.read((char*)&Tcw.at<float>(1, 3), sizeof(float));
    f.read((char*)&Tcw.at<float>(2, 3), sizeof(float));
    Tcw.at<float>(3,3) = 1.;
    cv::Mat Qcw(1,4, CV_32F);
    f.read((char*)&Qcw.at<float>(0, 0), sizeof(float));
    f.read((char*)&Qcw.at<float>(0, 1), sizeof(float));
    f.read((char*)&Qcw.at<float>(0, 2), sizeof(float));
    f.read((char*)&Qcw.at<float>(0, 3), sizeof(float));
    Converter::RmatOfQuat(Tcw, Qcw);
    fr.SetPose(Tcw);
    f.read((char*)&fr.N, sizeof(fr.N));
    fr.mvKeys.reserve(fr.N);
    fr.mDescriptors.create(fr.N, 32, CV_8UC1);
    fr.mvpMapPoints = vector<MapPoint*>(fr.N,static_cast<MapPoint*>(NULL));
    for (int i=0; i<fr.N; i++)
    {
        cv::KeyPoint kp;
        f.read((char*)&kp.pt.x,     sizeof(kp.pt.x));
        f.read((char*)&kp.pt.y,     sizeof(kp.pt.y));
        f.read((char*)&kp.size,     sizeof(kp.size));
        f.read((char*)&kp.angle,    sizeof(kp.angle));
        f.read((char*)&kp.response, sizeof(kp.response));
        f.read((char*)&kp.octave,   sizeof(kp.octave));
        fr.mvKeys.push_back(kp);
        for (int j=0; j<32; j++)
            f.read((char*)&fr.mDescriptors.at<unsigned char>(i, j), sizeof(char));
        unsigned long int mpidx;
        f.read((char*)&mpidx,   sizeof(mpidx));
        if (mpidx == ULONG_MAX)	fr.mvpMapPoints[i] = NULL;
        else fr.mvpMapPoints[i] = amp[mpidx];
    }

    fr.mvuRight = vector<float>(fr.N,-1);
    fr.mvDepth = vector<float>(fr.N,-1);
    fr.mpORBextractorLeft = orb_ext;
    fr.InitializeScaleLevels();
    fr.UndistortKeyPoints();
    fr.AssignFeaturesToGrid();
    fr.ComputeBoW();

    KeyFrame* kf = new KeyFrame(fr, this, NULL);
    kf->mnId = fr.mnId;

    for (int i=0; i<fr.N; i++)
    {
        if (fr.mvpMapPoints[i])
        {
            fr.mvpMapPoints[i]->AddObservation(kf, i);
            if (!fr.mvpMapPoints[i]->GetReferenceKeyFrame()) fr.mvpMapPoints[i]->SetReferenceKeyFrame(kf);
        }
    }

    return kf;
}


MapPoint* Map::_ReadMapPoint(ifstream &f)
{
    long unsigned int id;
    f.read((char*)&id, sizeof(id));
    cv::Mat wp(3,1, CV_32F);
    f.read((char*)&wp.at<float>(0), sizeof(float));
    f.read((char*)&wp.at<float>(1), sizeof(float));
    f.read((char*)&wp.at<float>(2), sizeof(float));
    long int mnFirstKFid=0, mnFirstFrame=0;
    MapPoint* mp = new MapPoint(wp, mnFirstKFid, mnFirstFrame, this);
    mp->mnId = id;
    return mp;
}




bool Map::Load(const string &filename, ORBVocabulary &voc)
{
    if (!Camera::initialized)
    {
        cerr << "Map: camera is not initialized. Cowardly refusing to load anything" << endl;
        return false;
    }

    int nFeatures = 2000;
    float scaleFactor = 1.2;
    int nLevels = 8, fIniThFAST = 20, fMinThFAST = 7;
    ORB_SLAM2::ORBextractor orb_ext = ORB_SLAM2::ORBextractor(nFeatures, scaleFactor, nLevels, fIniThFAST, fMinThFAST);

    cerr << "Map: reading from " << filename << endl;
    ifstream f;
    f.open(filename.c_str());

    long unsigned int nb_mappoints, max_id=0;
    f.read((char*)&nb_mappoints, sizeof(nb_mappoints));
    cerr << "reading " << nb_mappoints << " mappoints" << endl;
    for (unsigned int i=0; i<nb_mappoints; i++)
    {
        ORB_SLAM2::MapPoint* mp = _ReadMapPoint(f);
        if (mp->mnId>=max_id) max_id=mp->mnId;
        AddMapPoint(mp);
    }
    ORB_SLAM2::MapPoint::nNextId = max_id+1;
    std::vector<MapPoint*> amp = GetAllMapPoints();
    long unsigned int nb_keyframes;
    f.read((char*)&nb_keyframes, sizeof(nb_keyframes));
    cerr << "reading " << nb_keyframes << " keyframe" << endl;
    vector<KeyFrame*> kf_by_order;
    for (unsigned int i=0; i<nb_keyframes; i++)
    {
        KeyFrame* kf = _ReadKeyFrame(f, voc, amp, &orb_ext);
        AddKeyFrame(kf);
        kf_by_order.push_back(kf);
    }
    ORB_SLAM2::KeyFrame::nNextId = mnMaxKFid +1;
    map<unsigned long int, KeyFrame*> kf_by_id;
    for(auto kf: mspKeyFrames)
        kf_by_id[kf->mnId] = kf;

    for(auto kf: kf_by_order)
    {
        unsigned long int parent_id;
        f.read((char*)&parent_id, sizeof(parent_id));
        if (parent_id != ULONG_MAX)
        kf->ChangeParent(kf_by_id[parent_id]);
        unsigned long int nb_con;
        f.read((char*)&nb_con, sizeof(nb_con));
        for (unsigned long int i=0; i<nb_con; i++)
        {
            unsigned long int id; int weight;
            f.read((char*)&id, sizeof(id));
            f.read((char*)&weight, sizeof(weight));
            kf->AddConnection(kf_by_id[id], weight);
        }
    }

    for(auto mp: amp)
    {
        mp->ComputeDistinctiveDescriptors();
        mp->UpdateNormalAndDepth();
    }
    return true;
}



void Map::_WriteMapPoint(ofstream &f, MapPoint* mp)
{
    f.write((char*)&mp->mnId, sizeof(mp->mnId));
    cv::Mat wp = mp->GetWorldPos();
    f.write((char*)&wp.at<float>(0), sizeof(float));
    f.write((char*)&wp.at<float>(1), sizeof(float));
    f.write((char*)&wp.at<float>(2), sizeof(float));
}


void Map::_WriteKeyFrame(ofstream &f, KeyFrame* kf, map<MapPoint*, unsigned long int>& idx_of_mp)
{
    f.write((char*)&kf->mnId, sizeof(kf->mnId));
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));
    cv::Mat Tcw = kf->GetPose();
    f.write((char*)&Tcw.at<float>(0,3), sizeof(float));
    f.write((char*)&Tcw.at<float>(1,3), sizeof(float));
    f.write((char*)&Tcw.at<float>(2,3), sizeof(float));
    vector<float> Qcw = Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    f.write((char*)&Qcw[0], sizeof(float));
    f.write((char*)&Qcw[1], sizeof(float));
    f.write((char*)&Qcw[2], sizeof(float));
    f.write((char*)&Qcw[3], sizeof(float));
    f.write((char*)&kf->N, sizeof(kf->N));
    for (int i=0; i<kf->N; i++)
    {
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x,     sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y,     sizeof(kp.pt.y));
        f.write((char*)&kp.size,     sizeof(kp.size));
        f.write((char*)&kp.angle,    sizeof(kp.angle));
        f.write((char*)&kp.response, sizeof(kp.response));
        f.write((char*)&kp.octave,   sizeof(kp.octave));
        for (int j=0; j<32; j++)
            f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));

        unsigned long int mpidx; MapPoint* mp = kf->GetMapPoint(i);
        if (mp == NULL) mpidx = ULONG_MAX;
        else mpidx = idx_of_mp[mp];
        f.write((char*)&mpidx,   sizeof(mpidx));
    }

}

bool Map::Save(const string &filename)
{
    cerr << "Map: Saving to " << filename << endl;
    ofstream f;
    f.open(filename.c_str(), ios_base::out|ios::binary);

    cerr << "  writing " << mspMapPoints.size() << " mappoints" << endl;
    unsigned long int nbMapPoints = mspMapPoints.size();
    f.write((char*)&nbMapPoints, sizeof(nbMapPoints));
    for(auto mp: mspMapPoints)
    _WriteMapPoint(f, mp);

    map<MapPoint*, unsigned long int> idx_of_mp;
    unsigned long int i = 0;
    for(auto mp: mspMapPoints)
    {
        idx_of_mp[mp] = i;
        i += 1;
    }

    cerr << "  writing " << mspKeyFrames.size() << " keyframes" << endl;
    unsigned long int nbKeyFrames = mspKeyFrames.size();
    f.write((char*)&nbKeyFrames, sizeof(nbKeyFrames));
    for(auto kf: mspKeyFrames)
        _WriteKeyFrame(f, kf, idx_of_mp);

    // store tree and graph
    for(auto kf: mspKeyFrames)
    {
        KeyFrame* parent = kf->GetParent();
        unsigned long int parent_id = ULONG_MAX;
        if (parent) parent_id = parent->mnId;
        f.write((char*)&parent_id, sizeof(parent_id));
        unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
        f.write((char*)&nb_con, sizeof(nb_con));
        for (auto ckf: kf->GetConnectedKeyFrames())
        {
            int weight = kf->GetWeight(ckf);
            f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
            f.write((char*)&weight, sizeof(weight));
        }
    }

    f.close();
    cerr << "Map: finished saving" << endl;
    struct stat st;
    stat(filename.c_str(), &st);
    cerr << "Map: saved " << st.st_size << " bytes" << endl;
    return true;

}
} //namespace ORB_SLAM2
