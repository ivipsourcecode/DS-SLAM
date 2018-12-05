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

#ifndef PANGOLIN_VIEWER_H
#define PANGOLIN_VIEWER_H

#include "Viewer.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"

#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class PangolinViewer : public Viewer
{
public:
    PangolinViewer(const string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();
    void UpdateFrame(Tracking *pTracker);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void Register(System* pSystem);
    void RegisterMap(Map* map);
    void Finalize(void);

private:
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
};

}
#endif // PANGOLIN_VIEWER_H
	

