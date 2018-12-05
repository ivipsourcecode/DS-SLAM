/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
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


#include "pointcloudmapping.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h> 

pcl::PointCloud<pcl::PointXYZRGBA> pcl_filter; 
ros::Publisher pclPoint_pub;
ros::Publisher octomap_pub;
sensor_msgs::PointCloud2 pcl_point;

pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud_kf;

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    this->sor.setMeanK(50);                                
    this->sor.setStddevMulThresh(1.0);                    
    globalMap = boost::make_shared< PointCloud >( );
    KfMap = boost::make_shared< PointCloud >( );
    viewerThread = boost::make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);  
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& semantic_color,cv::Mat& semantic,cv::Mat& color, cv::Mat& depth)
{
    
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    semanticImgs.push_back(semantic.clone() );
    semanticImgs_color.push_back(semantic_color.clone() );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& semantic_color,cv::Mat& semantic, cv::Mat& color, cv::Mat& depth)
{

    PointCloud::Ptr tmp( new PointCloud() );
    // Point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=1)
    {
        for ( int n=0; n<depth.cols; n+=1)
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d > 8)
                continue;
	            int flag_exist=0;
	     
		        for (int i=-20;i <= 20; i+=3)
		        {
                    for (int j=-20;j <= 20; j+=3)
                    {
                        int tempx = m + i;
                        int tempy = n + j ;
            
                        if( tempx <= 0  ) tempx = 0;
                        if( tempx >= (Camera::height -1)  ) tempx = Camera::height-1;
                        if( tempy  <= 0  ) tempy =  0;
                        if( tempy >= (Camera::width -1) ) tempy = Camera::width -1;
                        if((int)semantic.ptr<uchar>(tempx)[tempy] == PEOPLE_LABLE)  
                        {
                            flag_exist=1;
                            break;
                        }
                    }
                    if(flag_exist==1)
                        break;
		        }
	
                if(flag_exist == 1)
                    continue;

                PointT p;
                p.z = d;
                p.x = ( n - Camera::cx) * p.z / Camera::fx;
                p.y = ( m - Camera::cy) * p.z / Camera::fy;

	            // Deal with color
	            if((int)semantic.ptr<uchar>(m)[n]==0)
	            {
                    p.b = color.ptr<uchar>(m)[n*3];
                    p.g = color.ptr<uchar>(m)[n*3+1];
                    p.r = color.ptr<uchar>(m)[n*3+2];
	            }
	            else
	            {
                    p.b = semantic_color.ptr<uchar>(m)[n*3];
                    p.g = semantic_color.ptr<uchar>(m)[n*3+1];
                    p.r = semantic_color.ptr<uchar>(m)[n*3+2]; 
	            }
	            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    cout<<"Generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


void PointCloudMapping::viewer()
{

    ros::NodeHandle n;
    pclPoint_pub = n.advertise<sensor_msgs::PointCloud2>("/ORB_SLAM2_PointMap_SegNetM/Point_Clouds",100000);
    ros::Rate r(5);
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(N==0)
	    {
	        cout<<"Keyframes miss!"<<endl;
            usleep(1000);
	        continue;
	    }
        KfMap->clear();
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i],semanticImgs_color[i], semanticImgs[i],colorImgs[i], depthImgs[i] );
	        *KfMap += *p;
	        *globalMap += *p;	    
        }
	
	    PointCloud::Ptr tmp1(new PointCloud());
        voxel.setInputCloud( KfMap );
        voxel.filter( *tmp1 );
        KfMap->swap( *tmp1 );	
        pcl_cloud_kf = *KfMap;	

	    Cloud_transform(pcl_cloud_kf,pcl_filter);
	    pcl::toROSMsg(pcl_filter, pcl_point);
	    pcl_point.header.frame_id = "/pointCloud";
	    pclPoint_pub.publish(pcl_point);
        lastKeyframeSize = N;
	    cout << "Keyframe map publish time ="<<endl;
    }

}

void PointCloudMapping::public_cloud( pcl::PointCloud< pcl::PointXYZRGBA >& cloud_kf )
{
	cloud_kf =pcl_cloud_kf; 
}

void PointCloudMapping::Cloud_transform(pcl::PointCloud<pcl::PointXYZRGBA>& source, pcl::PointCloud<pcl::PointXYZRGBA>& out)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered;
	Eigen::Matrix4f m;

	m<< 0,0,1,0,
	    -1,0,0,0,
		0,-1,0,0;
	Eigen::Affine3f transform(m);
	pcl::transformPointCloud (source, out, transform);
}
