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


#ifndef VIEWER_H
#define VIEWER_H

#include "Tracking.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class System;

class Viewer
{

public:
    Viewer();

    virtual void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    virtual void UpdateFrame(Tracking *pTracker);
    virtual void SetCurrentCameraPose(const cv::Mat &Tcw);
    virtual void Register(System* pSystem);
    virtual void Finalize(void);

protected:
    System* mpSystem;
    bool mbFinished;
    bool Stop();
    bool CheckFinish();
    void SetFinish();


private:
    bool mbFinishRequested;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};

}


#endif // VIEWER_H
	

