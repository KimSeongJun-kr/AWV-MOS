# *AWV-MOS-LIO*
AWV-MOS-LIO: Adaptive Window Visibility based Moving Object Segmentation with LiDAR Inertial Odometry
## 1. Introduction
AWV-MOS is **a LiDAR Moving Object Segmentation module** for online MOS and static map construction with external pose data.  
AWV-MOS-LIO is **a Online LiDAR Moving Object Segmentation system integrated with LiDAR Inertial Odometry**.  
- This repository contains AWV-MOS package and AWV-MOS-LIO package.
- AWV-MOS package provides online MOS and static mapping using external poses data 
- AWV-MOS-LIO package provides online MOS using integrated LIO system.
- The dataset used for paper experiments and our results are provided from the NAS server.
#### Online MOS demonstration video is available at [youtube](https://youtu.be/LbfhA7Kv76c)

### Table of Contents
1. [Introduction](#1-Introduction)
2. [Requirements](#2-Requirements)
3. [Install](#3-Install)
4. [How to use](#4-How-to-use)
5. [Examples](#5-Examples)
6. [Evaluation](#6-Evaluation)
7. [Downloads](#7-Downloads)
8. [Citation](#8-citation)

## 2. Requirements
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html), [PCL](https://pointclouds.org/downloads/), [OpenCV](https://opencv.org/get-started/), [TBB](https://github.com/oneapi-src/oneTBB/blob/master/INSTALL.md), 
    ```
    $ sudo apt-get update
    $ sudo apt-get install -y libeigen3-dev libpcl-dev libopencv-dev libtbb-dev
    ```
- [gtsam](https://gtsam.org/get_started/)
    ```
    $ sudo apt-get install -y software-properties-common
    $ sudo add-apt-repository ppa:borglab/gtsam-release-4.0
    $ sudo apt-get update
    $ sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
    ```
- [ROS](http://wiki.ros.org/ROS/Installation) (tested with noetic).
    ```
    $ sudo apt-get install -y curl lsb-release gnupg g++
    $ sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
    $ sudo apt-get update
    $ sudo apt-get install -y ros-noetic-desktop ros-noetic-pcl-conversions
    ```

## 3. Install
Use the following commands to download and compile the package.
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/KimSeongJun-kr/AWV-MOS.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DUSE_EVALUATION_POINT_TYPE=OFF
```

## 4. How to use
If you want to run the package on your own data, you should setup parameters in '*config/params_custom.yaml*' to match your sensor configuration. 
### **Online MOS with LIO system**:  
Need to publish LiDAR scan and IMU data messages. The LiDAR scan points should contain time field.
```
$ roslaunch awv_mos_lio run_online_mos_w_lio.launch config_file_name:=<your_config_file.yaml>
```
### **Online MOS with external SLAM system**:  
Need to publish deskewed LiDAR scan messages and sensor pose messages of type 'nav_msgs/Odometry'.
```
$ roslaunch awv_mos run_online_mos.launch scan_topic:=<scan_topic_name> pose_topic:=<pose_topic_name> config_file_name:=<your_config_file.yaml>
```
### **Static map construction with bag file**:  
The input bag file should contain deskewed LiDAR scan and sensor pose messages of type 'nav_msgs/Odometry'.  
If the scan data is not deskewed but includes a time field, the deskewing feature is available by setting *use_deskewing* to true.
```
$ roslaunch awv_mos run_static_mapping_bag.launch bag_file_path:=<path/to/your_bag.bag> scan_topic:=<scan_topic_name> pose_topic:=<pose_topic_name> config_file_name:=<your_config_file.yaml> use_deskewing:=false
```  
### **Static map construction with KITTI format data**:
Need to construct KITTI folder structure and format as follows:
```
/dataset/
    └── sequences  
        ├── 00  
        │   ├── poses.txt  
        │   ├── times.txt  
        │   ├── calib.txt  
        │   ├── labels  
        │   │   ├── 000000.label  
        │   │   ├── 000001.label  
        │   └── velodyne  
        │       ├── 000000.bin  
        │       ├── 000001.bin  
```
```
$ roslaunch awv_mos run_static_mapping_kitti.launch path_to_sequences:=<path/to/sequences> sequence:=<target_seqeucne_number> config_file_name:=<your_config_file.yaml>
```

## 5. Examples
Example datas can be downloaded at [Downloads](#Downloads)
- Online MOS with LIO system on KITTI Raw dataset:  
```
$ roslaunch awv_mos_lio run_online_mos_w_lio.launch config_file_name:=params_kitti.yaml
$ rosbag play kitti_raw_sequence_01_date_2011_10_03_drive_0042.bag
```
- Online MOS with external SLAM system on nuscenes dataset:  
To run nuscenes dataset example, [FAST_LIO](https://github.com/hku-mars/M-detector/releases/tag/FAST_LIO) is required.
```
$ roslaunch awv_mos run_online_mos.launch scan_topic:=/LIDAR_TOP pose_topic:=/aft_mapped_to_init config_file_name:=params_nuscenes.yaml
$ roslaunch fast_lio mapping_nuscenes.launch
$ rosbag play NuScenes-v1.0-trainval-scene-1059.bag --topics /LIDAR_TOP --wait-for-subscribers
```
- Static map construction with bag file that contains deskewed or skewed KITTI Raw data:  
```
// (deskewed scan)
$ roslaunch awv_mos run_static_mapping_bag.launch bag_file_path:=s08_awv-mos-lio_results.bag scan_topic:=/awv_mos/segmented_query_scan_all pose_topic:=/lio_sam/mapping/odometry_incremental config_file_name:=params_kitti_mapping.yaml use_deskewing:=false
```
```
// (skewed scan)
$ roslaunch awv_mos run_static_mapping_bag.launch bag_file_path:=s08_awv-mos-lio_results.bag scan_topic:=/velodyne_points pose_topic:=/lio_sam/mapping/odometry_incremental config_file_name:=params_kitti_mapping.yaml use_deskewing:=true
```

## 6. Evaluation
### Evaluate online MOS with LIO. (AWS-MOS-LIO)
Due to point loss occurring in the LIO-SAM part, we evaluate AWV-MOS-LIO using scan messages that include label field.  
The bag file contains label field-included scan messages is generated by [kittiraw2bag](https://github.com/KimSeongJun-kr/kittiraw2bag.git).  
1. build with *USE_EVALUATION_POINT_TYPE* macro option
```
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DUSE_EVALUATION_POINT_TYPE=ON
```
2. Recored the MOS result:
```
$ rosbag record /awv_mos/segmented_query_scan_all
```
3. Run the launch file & Play existing bag files:
```
$ roslaunch awv_mos run_kitti.launch
$ rosbag play kitti_raw_sequence_01_date_2011_10_03_drive_0042.bag
```
4. Evaluate the results using evaluate_mos.launch:  
```
$ roslaunch awv_mos mos_evaluation.launch use_bag_file:=true bag_folder_path:=<path/to/bagfile_folder> bag_file_name:=s01_awv-mos-lio_results.bag
```
&emsp;&emsp;Note that, as we showed in the paper, in the case of the KITTI Raw 08 sequence, we skip the first 150 frames because there is a stopped car that is labeled as moving.
```
$ roslaunch awv_mos mos_evaluation.launch use_bag_file:=true bag_folder_path:=<path/to/bagfile_folder> bag_file_name:=s08_awv-mos-lio_results.bag start_frame:=150
```
### Evaluate online MOS with external pose data. (AWV-MOS)
1. Set the *m_cfg_b_use_prediction_write* to *true* in configuration file.
2. Run **Online MOS with external SLAM system**
```
$ roslaunch awv_mos run_online_mos.launch scan_topic:=/LIDAR_TOP pose_topic:=/aft_mapped_to_init config_file_name:=params_nuscenes.yaml
```
3. Evaluate the results using evaluation_mos.launch
```
$ roslaunch awv_mos mos_evaluation.launch use_label_file:=true gt_label_folder_path:=<path/to/gt_label_folder> pred_label_folder_path:=<path/to/pred_label_folder>
```
### Evaluate static map construction. (AWV-MOS)
1. build with *USE_EVALUATION_POINT_TYPE* macro option
```
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DUSE_EVALUATION_POINT_TYPE=ON
```
2. Run the launch
In case of KITTI Raw 08 sequence, set *m_cfg_i_mapping_start_frame_limit* to 150.
```
$ roslaunch awv_mos run_static_mapping_bag.launch bag_file_path:=s08_awv-mos-lio_results.bag scan_topic:=/velodyne_points pose_topic:=/lio_sam/mapping/odometry_incremental config_file_name:=params_kitti_mapping.yaml use_deskewing:=true
```
3. The evaluation result of the map will be printed during the mapping process.

## 7. Downloads
- KITTI Raw dataset ROS Bag files used for paper experiments can be downloaded from [the NAS server](https://gofile.me/6Vvuz/njqrKwtxs)
- The uploaded ROS bag files of KITTI Raw are generated using [kittiraw2bag](https://github.com/KimSeongJun-kr/kittiraw2bag.git), which generates scan messages that include the ring and time fields.

## 8. Citation
- To be updated