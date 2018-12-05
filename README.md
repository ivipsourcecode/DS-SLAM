# **DS-SLAM**

DS-SLAM is a complete robust semantic SLAM system, which could reduce the influence of dynamic objects on pose estimation, such as walking people and other moving robots. Meanwhile, DS-SLAM could also provide semantic presentation of the octo-tree map．DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2 (from https://github.com/raulmur/ORB_SLAM2 and https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map, thanks for Raul's, Gao Xiang's and SegNet's great work!). It not only includes tracking, local mapping, loop closing threads, but also contains semantic segmentation, dense mapping threads. Currently, the system is integrated with Robot Operating System (ROS). In this Open Source project, we provide one example to run DS_SLAM in the TUM dataset with RGB-D sensors. The example of real-time implementation of DS-SLAM and hardware/software configurations would be coming soon.

As described in **DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments **Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei  Qiao, Published in the Proceedings of  the 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2018) 

Moreover, to download the pre-print version, you could be directed to [https://arxiv.org/abs/1809.08379v1].

DS-SLAM is developed by the joint research project of iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/) and Advanced Mechanism and Roboticized Equipment Lab.

If you have any questions or use DS_SLAM for commercial purposes, please contact: qiaofei@tsinghua.edu.cn

# 1. License

DS-SLAM is released under a  [GPLv3 license](https://github.com/zoeyuchao/DS-SLAM/blob/master/LICENSE).

DS-SLAM allows personal and research use only. For a commercial license please contact: qiaofei@tsinghua.edu.cn

If you use DS-SLAM in an academic work, please cite their publications as below:

Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei Qiao "DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments." Published in the Proceedings of the 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2018). 

# 2. Prerequisites

We have tested the library in Ubuntu 14.04 and 16.04, but it should be easy to compile in other platforms. The experiment is performed on a computer with Intel i7 CPU, P4000 GPU, and 32GB memory.

### ORB_SLAM2 Prereguisites

DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2. In order to run DS_SLAM, you have to install environment needed by ORB_SLAM2(the section of 2. Prereguisites). We suggest that the path of the folder of Thirdparty and Vocabulary(provided at <https://pan.baidu.com/s/1-zWXlzOn-X0tjoEF9XI6qA>, extract code: 8t7x) to be DS-SLAM. Instructions can be found at: https://github.com/raulmur/ORB_SLAM2.

### ROS

We provide one example to process the TUM dataset as RGB-D image. A version Hydro or newer is needed. You should create a ROS catkin workspace(in our case, catkin_ws).

### SegNet

We adopt SegNet to provide pixel-wise semantic segmentation based on caffe in real-time. Download and decompress DS_SLAM library (provided at https://github.com/zoeyuchao/DS-SLAM) in catkin_ws/src. Then you can find the Example folder in the folder of DS-SLAM. We suggest the installation path of caffe-segnet to be /Examples/ROS/ORB_SLAM2_PointMap_SegNetM. The root of Download and install instructions can be found at: https://github.com/TimoSaemann/caffe-segnet-cudnn5

### OctoMap and RVIZ

We provide semantic presentation of the octo-tree map by OctoMap. RViz display plugins for visualizing octomap messages. We suggest that the installation path of octomap_mapping and octomap_rviz_plugins to be catkin_ws/src. Add #define COLOR_OCTOMAP_SERVER into the OctomapServer.h at the folder of  octomap_mapping/octomap_server/include/octomap_server Download and install instructions can be found at: https://github.com/OctoMap/octomap_mapping and https://github.com/OctoMap/octomap_rviz_plugins.    

# 3. Building DS_SLAM library and the example

We provide a script DS_SLAM_BUILD.sh to build the third party libraries and DS-SLAM. Please make sure you have installed all previous required dependencies. Execute:

```c++
cd DS-SLAM
chmod +x DS_SLAM_BUILD.sh
./DS_SLAM_BUILD.sh
```

# 4. TUM example

1. Add the path including Examples/ROS/ORB_SLAM2_PointMap_SegNetM to the ROS_PACKAGE_PATH            environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2:

```
export  ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/DS-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM
```

2. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and unzip it. We suggest you download rgbd_dataset_freiburg3_walking_xyz.
3. We provide DS_SLAM_TUM.launch script to run TUM example. Change PATH_TO_SEQUENCE and  PATH_TO_SEQUENCE/associate.txt in the DS_SLAM_TUM.launch to the sequence directory that you download before, then  execute the following command in a new terminal. Execute:

```
cd DS-SLAM
roslaunch DS_SLAM_TUM.launch 
```

#  5. Something about Folders

The function of folder in the catkin_ws/src/ORB_SLAM2_PointMap_SegNetM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM.

1. segmentation: the section of segmentation including source code, header file and dynamic link library created by Cmake.
2. launch: used for showing octomap.
3. prototxts and tools: containing some parameters associated with caffe net relating to the semantic segmentation thread of DS_SLAM. There is the folder of models(provided at https://pan.baidu.com/s/1gkI7nAvijF5Fjj0DTR0rEg extract code: fpa3), please download and place the folder in the same path with the folder of prototxts and tools.





