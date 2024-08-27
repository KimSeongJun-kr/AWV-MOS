#include "AWV_MOS.h"
#include <filesystem>
#include <termios.h>
#include <unistd.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>


bool ReadCalibFile(const std::string& i_file_path, Eigen::Affine3f& o_tf_lidar_to_base) {
    std::ifstream calib_file(i_file_path);
    if (!calib_file.is_open()) {
        std::cerr << "Could not open calib file: " << i_file_path << std::endl;
        return false;
    }

    Eigen::Affine3f tf_lidar_to_base;
    std::string line;
    while (std::getline(calib_file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (!(iss >> key)) continue;

        std::vector<float> values((std::istream_iterator<float>(iss)), std::istream_iterator<float>());
        if (values.size() != 12) continue;

        Eigen::Matrix4f matrix;
        matrix << values[0], values[1], values[2], values[3],
                  values[4], values[5], values[6], values[7],
                  values[8], values[9], values[10], values[11],
                  0, 0, 0, 1;

        tf_lidar_to_base = Eigen::Affine3f(matrix);
    }
    o_tf_lidar_to_base = tf_lidar_to_base;

    calib_file.close();

    std::cout << "Read calibration file done. \n";
    return true;
}

bool ReadPosesFile(const std::string& i_file_path, const Eigen::Affine3f& i_tf_lidar_to_base, std::vector<Eigen::Affine3f>& o_poses) 
{
    std::ifstream pose_file(i_file_path);
    if (!pose_file.is_open()) {
        std::cerr << "Could not open poses file: " << i_file_path << std::endl;
        return false;
    }

    std::vector<Eigen::Affine3f> poses;
    Eigen::Affine3f tf_cam_coordi_system_to_world_coordi_system = i_tf_lidar_to_base.inverse();
    std::string line;
    while (std::getline(pose_file, line)) {
        std::istringstream iss(line);
        std::vector<float> values;
        float value;
        while (iss >> value) {
            values.push_back(value);
        }

        // 각 포즈 데이터는 12개의 값으로 구성되어 있다고 가정합니다 (3x4 행렬).
        if (values.size() != 12) {
            std::cerr << "Incorrect format in line: " << line << std::endl;
            continue;
        }

        Eigen::Matrix4f matrix;
        matrix << values[0], values[1], values[2], values[3],
                  values[4], values[5], values[6], values[7],
                  values[8], values[9], values[10], values[11],
                  0, 0, 0, 1;

        Eigen::Affine3f tf_base_to_origin = Eigen::Affine3f(matrix);

        poses.push_back(tf_cam_coordi_system_to_world_coordi_system * tf_base_to_origin * i_tf_lidar_to_base);
    }
    pose_file.close();
    o_poses = poses;

    std::cout << "Read poses file done. poses size: " << o_poses.size() << "\n";

    return true;
}

bool ReadTimeFile(const std::string& i_file_path, std::vector<double>& o_times)
{
    std::ifstream time_file(i_file_path);
    if (!time_file.is_open()) {
        std::cerr << "Could not open times file: " << i_file_path << std::endl;
        return false;
    }

    std::vector<double> times;
    std::string line;
    while (std::getline(time_file, line)) {
        double number = std::stod(line);
        if (number == 0.0) {
            number = 0.0001;
        }
        times.push_back(number);
    }
    time_file.close();
    o_times = times;

    std::cout << "Read times file done. poses size: " << o_times.size() << "\n";
    return true;
}

int extract_number(const std::string& i_file_path) {
    // Get file name
    size_t last_slash_idx = i_file_path.find_last_of("/\\");
    size_t expender_idx = i_file_path.find_last_of(".");
    std::string file_name;
    if (last_slash_idx != std::string::npos && expender_idx != std::string::npos)
    {
        file_name = i_file_path.substr(last_slash_idx + 1, expender_idx - last_slash_idx - 1);
        return std::stoi(file_name);
    }
    else
        return -1;
}

bool GetFileList(const std::string& i_folder_path, std::vector<std::string>& o_file_list)
{
    std::vector<std::string> file_list;

    if (std::filesystem::exists(i_folder_path) == false || 
        std::filesystem::is_directory(i_folder_path) == false)
    {
        std::cout << "Scan file folder does not exist or is not a directory: " << i_folder_path << std::endl;
        return false;
    }
        
    for (const auto& entry : std::filesystem::directory_iterator(i_folder_path)) 
    {
        if (entry.is_regular_file()) 
        {
            file_list.push_back(entry.path().string());
        }
    }
 
    std::sort(file_list.begin(), file_list.end(), 
            [](const std::string& a, const std::string& b) 
            {return extract_number(a) < extract_number(b);});
    
    o_file_list = file_list;

    std::cout << "Get scan file list done. list size: " << o_file_list.size() << "\n";
    return true;
}

bool ReadScanFileWithLabel(const std::string& i_scan_file_path, const std::string& i_label_file_path, pcl::PointCloud<PointTypeMOS>::Ptr& o_scan)
{
    std::ifstream label_file(i_label_file_path, std::ios::binary);
    if (!label_file.is_open()) {
        std::cerr << "Failed to open file: " << i_label_file_path << std::endl;
        return false;
    }

    std::vector<uint16_t> sem_labels;
    uint32_t label;
    while (label_file.read(reinterpret_cast<char*>(&label), sizeof(uint32_t))) {
        uint16_t sem_label = label & 0xFFFF;
        uint16_t inst_label = label >> 16;
        sem_labels.push_back(sem_label);
    }
    label_file.close();

    std::ifstream scan_file(i_scan_file_path, std::ios::binary);
    if (!scan_file.is_open()) {
        std::cerr << "Failed to open file: " << i_scan_file_path << std::endl;
        return false;
    }

    pcl::PointCloud<PointTypeMOS>::Ptr scan(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
    int idx = 0;
    float buffer[4]; // Buffer for a single point
    while (scan_file.read(reinterpret_cast<char*>(&buffer), 16)) { // Read 16 bytes for each point
        PointTypeMOSEval point_eval;
        point_eval.x = buffer[0];
        point_eval.y = buffer[1];
        point_eval.z = buffer[2];
        point_eval.intensity = buffer[3];
        point_eval.label = sem_labels[idx];
        idx++;

        PointTypeMOS point;
        pcl::copyPoint(point_eval, point);
        scan->push_back(point);
    }
    scan_file.close();

    o_scan = scan;

    return true;
}

bool ReadScanFile(const std::string& i_file_path, pcl::PointCloud<PointTypeMOS>::Ptr& o_scan)
{
    std::ifstream scan_file(i_file_path, std::ios::binary);
    if (!scan_file.is_open()) {
        std::cerr << "Failed to open file: " << i_file_path << std::endl;
        return false;
    }

    pcl::PointCloud<PointTypeMOS>::Ptr scan(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
    float buffer[4]; // Buffer for a single point
    while (scan_file.read(reinterpret_cast<char*>(&buffer), 16)) { // Read 16 bytes for each point
        PointTypeMOS point;
        point.x = buffer[0];
        point.y = buffer[1];
        point.z = buffer[2];
        point.intensity = buffer[3];
        scan->push_back(point);
    }
    scan_file.close();

    o_scan = scan;

    return true;
}

bool ReadDatasFromKittiFormat(AWV_MOS& i_awv_mos,
                            const std::string& i_calib_file_path, 
                            const std::string& i_poses_file_path, 
                            const std::string& i_scan_folder_path, 
                            const std::string& i_label_folder_path, 
                            const std::string& i_times_file_path,
                            std::vector<pcl::PointCloud<PointTypeMOS>::Ptr>& o_scans,
                            std::vector<Eigen::Affine3f>& o_poses,
                            std::vector<double>& o_times_s,
                            std::vector<int>& o_frames_id)
{

    Eigen::Affine3f tf_lidar_to_base;
    if(i_calib_file_path != "")
        ReadCalibFile(i_calib_file_path, tf_lidar_to_base);
    else
        tf_lidar_to_base = Eigen::Affine3f::Identity();

    // Read poses.txt
    std::vector<Eigen::Affine3f> poses;
    if(ReadPosesFile(i_poses_file_path, tf_lidar_to_base, poses) == false) {return -1;}
    
    // Get scan file list
    std::vector<std::string> scan_files;
    if(GetFileList(i_scan_folder_path, scan_files) == false) {return -1;}

    // Get label file list
    std::vector<std::string> label_files;
    if(isEvaluationPointType == true)
    {
        if(GetFileList(i_label_folder_path, label_files) == false) {return -1;}
    }

    // Read times.txt
    std::vector<double> times_s;
    if(ReadTimeFile(i_times_file_path, times_s) == false) {return -1;}

    // Set frames id
    std::vector<int> frames_id;
    for(int i = 0; i < poses.size(); i++)
        frames_id.push_back(i);

    // Validation Check
    if(scan_files.size() != times_s.size() && label_files.size() != times_s.size() && poses.size() != times_s.size())
    {
        std::cout << "scan files, poses, times size are not mached. scans size: " << scan_files.size() << ", labels size: " << label_files.size() << ", poses size: " << poses.size() << ", times size: " << times_s.size() << "\n";
        return false;
    }

    // Limit frames
    {
        std::vector<Eigen::Affine3f> tmp_poses;
        std::vector<std::string> tmp_scan_files;
        std::vector<std::string> tmp_label_files;
        std::vector<double> tmp_times_s;
        std::vector<int> tmp_frames_id;
        if(i_awv_mos.m_cfg_i_mapping_start_frame_limit > 0 || i_awv_mos.m_cfg_i_mapping_end_frame_limit > 0)
        {
            int start_frame_idx = 0;
            int end_frame_idx = poses.size() - 1;
            if(i_awv_mos.m_cfg_i_mapping_start_frame_limit > 0)
                start_frame_idx = i_awv_mos.m_cfg_i_mapping_start_frame_limit;
            if(i_awv_mos.m_cfg_i_mapping_end_frame_limit > 0)
                end_frame_idx = i_awv_mos.m_cfg_i_mapping_end_frame_limit;
            for(int idx = start_frame_idx; idx <= end_frame_idx; idx++)
            {
                tmp_poses.push_back(poses[idx]);
                tmp_scan_files.push_back(scan_files[idx]);
                if(isEvaluationPointType == true)
                    tmp_label_files.push_back(label_files[idx]);
                tmp_times_s.push_back(times_s[idx]);
                tmp_frames_id.push_back(frames_id[idx]);
            }
            poses = tmp_poses;
            scan_files = tmp_scan_files;
            label_files = tmp_label_files;
            times_s = tmp_times_s;
            frames_id = tmp_frames_id;
            std::cout << "Limit frames. scans size: " << scan_files.size() << ", labels size: " << label_files.size() << ", poses size: " << poses.size() << ", times size: " << times_s.size() << "\n";
        }
    }

    // Select files to use
    {
        std::vector<Eigen::Affine3f> tmp_poses;
        std::vector<std::string> tmp_scan_files;
        std::vector<std::string> tmp_label_files;
        std::vector<double> tmp_times_s;
        std::vector<int> tmp_frames_id;
        if(i_awv_mos.m_cfg_b_mapping_use_only_keyframes == true)
        {
            tmp_poses.push_back(poses[0]);
            tmp_scan_files.push_back(scan_files[0]);
            if(isEvaluationPointType == true)
                tmp_label_files.push_back(label_files[0]);
            tmp_times_s.push_back(times_s[0]);
            tmp_frames_id.push_back(frames_id[0]);

            for(int idx = 1; idx < poses.size(); idx++)
            {
                if(i_awv_mos.KeyframeSelection(tmp_poses.back(), tmp_times_s.back(), poses[idx], times_s[idx]) == true)
                {
                    tmp_poses.push_back(poses[idx]);
                    tmp_scan_files.push_back(scan_files[idx]);
                    if(isEvaluationPointType == true)
                        tmp_label_files.push_back(label_files[idx]);
                    tmp_times_s.push_back(times_s[idx]);
                    tmp_frames_id.push_back(frames_id[idx]);
                }
            }
            poses = tmp_poses;
            scan_files = tmp_scan_files;
            if(isEvaluationPointType == true)
                label_files = tmp_label_files;
            times_s = tmp_times_s;
            frames_id = tmp_frames_id;                
            std::cout << "Mapping using keyframes. Select total " << poses.size() << " keyframes\n";
        }
        else
            std::cout << "Mapping using all frames. Select total " << poses.size() << " frames\n";
    }

    // Read scan files
    std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> scans;
    for(int idx = 0; idx < scan_files.size() && ros::ok(); idx++)
    {
        pcl::PointCloud<PointTypeMOS>::Ptr scan;
        if(isEvaluationPointType == true)
        {
            if(ReadScanFileWithLabel(scan_files[idx], label_files[idx], scan) == false) {return false;};
        }
        else
        {
            if(ReadScanFile(scan_files[idx], scan) == false) {return false;};
        }
        scans.push_back(scan);

        float progress = static_cast<float>(idx + 1) / scan_files.size() * 100.0f;
        std::cout << "\rRead scan files Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                << progress << "% (" << (idx+1) << "/" << scan_files.size() << ")" << std::flush;
    }
    std::cout << "\n";

    o_poses = poses;
    o_scans = scans;
    o_times_s = times_s;
    o_frames_id = frames_id;

    return true;
}

void FindInterpolatedPoses(const std::vector<double>& i_target_times_s, 
                            const std::vector<double>& i_pose_times_s, 
                            const std::vector<Eigen::Affine3f>& i_poses,
                            std::vector<Eigen::Affine3f>& o_interped_poses) {
    std::vector<double> clamped_times_s = i_target_times_s;
    for (double& t : clamped_times_s) {
        t = std::clamp(t, i_pose_times_s.front(), i_pose_times_s.back());
    }

    std::vector<Eigen::Affine3f> interped_poses(clamped_times_s.size());
    for (size_t i = 0; i < clamped_times_s.size(); ++i) {
        double target_time = clamped_times_s[i];
        
        auto lower = std::lower_bound(i_pose_times_s.begin(), i_pose_times_s.end(), target_time);
        int next_idx = std::min((int)std::distance(i_pose_times_s.begin(), lower), (int)i_pose_times_s.size()-1);
        int prev_idx = std::max(0, next_idx - 1);

        double prev_time = i_pose_times_s[prev_idx];
        double next_time = i_pose_times_s[next_idx];

        double ratio = (target_time - prev_time) / (next_time - prev_time + 1e-9);

        Eigen::Vector3f prev_translation = i_poses[prev_idx].translation();
        Eigen::Vector3f next_translation = i_poses[next_idx].translation();
        Eigen::Quaternionf prev_rotations(i_poses[prev_idx].rotation());
        Eigen::Quaternionf next_rotations(i_poses[next_idx].rotation());

        Eigen::Quaternionf interped_rotation = prev_rotations.slerp(ratio, next_rotations);
        Eigen::Vector3f interped_translation = prev_translation * (1 - ratio) + next_translation * ratio;

        Eigen::Affine3f interped_pose = Eigen::Affine3f::Identity();
        interped_pose.linear() = interped_rotation.toRotationMatrix();
        interped_pose.translation() = interped_translation;
        
        interped_poses[i] = interped_pose;
    }

    o_interped_poses = interped_poses;

    return;
}

void FindClosestPoses(const std::vector<double>& i_target_times_s, 
                    const std::vector<double>& i_pose_times_s, 
                    const std::vector<Eigen::Affine3f>& i_poses,
                    std::vector<Eigen::Affine3f>& o_closest_poses,
                    std::vector<double>& o_closest_pose_times,
                    std::vector<int>& o_closest_poses_index) 
{
    
    size_t num_targets = i_target_times_s.size();
    std::vector<Eigen::Affine3f> closest_poses(num_targets);
    std::vector<int> closest_poses_index(num_targets);
    std::vector<double> closest_pose_times(num_targets);

    for (size_t i = 0; i < num_targets; ++i) 
    {
        auto lower = std::lower_bound(i_pose_times_s.begin(), i_pose_times_s.end(), i_target_times_s[i]);
        size_t idx = std::distance(i_pose_times_s.begin(), lower);

        size_t closest_idx;
        if (idx == 0) {
            closest_idx = 0;
        } else if (idx >= i_pose_times_s.size()) {
            closest_idx = i_pose_times_s.size() - 1;
        } else {
            size_t left_idx = idx - 1;
            size_t right_idx = idx;
            double left_diff = std::abs(i_pose_times_s[left_idx] - i_target_times_s[i]);
            double right_diff = std::abs(i_pose_times_s[right_idx] - i_target_times_s[i]);
            closest_idx = (left_diff <= right_diff) ? left_idx : right_idx;
        }

        closest_poses[i] = i_poses[closest_idx];
        closest_pose_times[i] = i_pose_times_s[closest_idx];
        closest_poses_index[i] = closest_idx;
    }
    o_closest_poses = closest_poses;
    o_closest_pose_times = closest_pose_times;
    o_closest_poses_index = closest_poses_index;

    return;
}

bool ReadDatasFromBag(AWV_MOS& i_awv_mos,
                        const std::string& i_calib_file_path, 
                        const std::string& i_bag_file_path, 
                        const std::string& i_scan_topic, 
                        const std::string& i_odom_topic,
                        const bool& i_use_deskewing,
                        std::vector<pcl::PointCloud<PointTypeMOS>::Ptr>& o_scans,
                        std::vector<Eigen::Affine3f>& o_poses,
                        std::vector<double>& o_times_s,
                        std::vector<int>& o_frames_id)
{
    Eigen::Affine3f tf_lidar_to_base;
    if(i_calib_file_path != "")
        ReadCalibFile(i_calib_file_path, tf_lidar_to_base);
    else
        tf_lidar_to_base = Eigen::Affine3f::Identity();

    Eigen::Affine3f tf_cam_coordi_system_to_world_coordi_system = tf_lidar_to_base.inverse();

    std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> scans;
    std::vector<double> scan_times_s;
    std::vector<int> frames_id;
    std::vector<std::vector<double>> scan_point_times_s;

    rosbag::Bag bag(i_bag_file_path);
    if(bag.isOpen() == false)
    {
        std::cout << "Fail to open bag file : " << i_bag_file_path << "\n";
        return false;        
    }

    rosbag::View scan_view;
    scan_view.addQuery(bag, rosbag::TopicQuery(i_scan_topic));
    int scan_msgs_size = scan_view.size();
    int count = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, scan_view)
    {
        if(ros::ok() == false)
            break;

        sensor_msgs::PointCloud2::ConstPtr scan_msg_instance = msg.instantiate<sensor_msgs::PointCloud2>();
        if (scan_msg_instance != NULL)
        {
            count++;

            double scan_time = scan_msg_instance->header.stamp.toSec();
            if(i_use_deskewing == true)
            {
                pcl::PointCloud<PointTypeMOSDeskew>::Ptr tmp_scan(boost::make_shared<pcl::PointCloud<PointTypeMOSDeskew>>());
                pcl::fromROSMsg(*scan_msg_instance, *tmp_scan);
                pcl::PointCloud<PointTypeMOS>::Ptr scan(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                pcl::copyPointCloud(*tmp_scan, *scan);
                scans.push_back(scan);
                std::vector<double> point_times(tmp_scan->points.size());
                for(int i = 0; i < tmp_scan->points.size(); i++)
                    point_times[i] = tmp_scan->points[i].time + scan_time;
                scan_point_times_s.push_back(point_times);
            }
            else
            {
                pcl::PointCloud<PointTypeMOS>::Ptr scan(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                pcl::fromROSMsg(*scan_msg_instance, *scan);
                scans.push_back(scan);
            }
            scan_times_s.push_back(scan_time);
            frames_id.push_back(count-1);

            float progress = static_cast<float>(count) / scan_msgs_size * 100.0f;
            if((int)(progress * 1000) % 10 == 0)
                std::cout << "\rRead scan msgs Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                        << progress << "% (" << (count) << "/" << scan_msgs_size << ")" << std::flush;
        }
    }
    std::cout << "\n";

    if(scans.size() <= 0)
    {
        std::cout << "No scans in bag file. scan_topic: " << i_scan_topic << "\n";
        return false;
    }

    std::vector<double> bag_odom_times_s;
    std::vector<Eigen::Affine3f> bag_odom_poses;
    rosbag::View odom_view;
    odom_view.addQuery(bag, rosbag::TopicQuery(i_odom_topic));
    BOOST_FOREACH(rosbag::MessageInstance const msg, odom_view)
    {
        if(ros::ok() == false)
            break;

        nav_msgs::Odometry::ConstPtr odom_msg_instance = msg.instantiate<nav_msgs::Odometry>();
        if (odom_msg_instance != NULL)
        {
            Eigen::Affine3f tf_base_to_origin = Eigen::Affine3f::Identity();
            tf_base_to_origin.translation() << odom_msg_instance->pose.pose.position.x, 
                                            odom_msg_instance->pose.pose.position.y, 
                                            odom_msg_instance->pose.pose.position.z;
            Eigen::Quaternionf quat(odom_msg_instance->pose.pose.orientation.w, 
                                    odom_msg_instance->pose.pose.orientation.x, 
                                    odom_msg_instance->pose.pose.orientation.y, 
                                    odom_msg_instance->pose.pose.orientation.z);
            tf_base_to_origin.rotate(quat);

            bag_odom_poses.push_back(tf_cam_coordi_system_to_world_coordi_system * tf_base_to_origin * tf_lidar_to_base);
            bag_odom_times_s.push_back(odom_msg_instance->header.stamp.toSec());
        }
    }

    std::vector<Eigen::Affine3f> poses;
    std::vector<double> pose_times_s;
    std::vector<int> closest_poses_index;
    FindClosestPoses(scan_times_s, bag_odom_times_s, bag_odom_poses, poses, pose_times_s, closest_poses_index);

    if(poses.size() <= 0)
    {
        std::cout << "No poses in bag file. odom_topic: " << i_odom_topic << "\n";
        return false;
    }
    std::cout << "Read odoms from bag file done. poses size: " << poses.size() << "\n";

    bag.close();

    // Validation Check
    if(scans.size() != scan_times_s.size() && poses.size() != scan_times_s.size())
    {
        std::cout << "scan files, poses, times size are not mached. scans size: " << scans.size() << ", poses size: " << poses.size() << ", times size: " << scan_times_s.size() << "\n";
        return false;
    }

    // Limit frames
    {
        std::vector<Eigen::Affine3f> tmp_poses;
        std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> tmp_scans;
        std::vector<double> tmp_scan_times_s;
        std::vector<int> tmp_frames_id;
        std::vector<std::vector<double>> tmp_scan_point_times_s;
        if(i_awv_mos.m_cfg_i_mapping_start_frame_limit > 0 || i_awv_mos.m_cfg_i_mapping_end_frame_limit > 0)
        {
            int start_frame_idx = 0;
            int end_frame_idx = poses.size() - 1;
            if(i_awv_mos.m_cfg_i_mapping_start_frame_limit > 0)
                start_frame_idx = std::min(i_awv_mos.m_cfg_i_mapping_start_frame_limit, end_frame_idx);
            if(i_awv_mos.m_cfg_i_mapping_end_frame_limit > 0)
                end_frame_idx = std::min(i_awv_mos.m_cfg_i_mapping_end_frame_limit, end_frame_idx);
            for(int idx = start_frame_idx; idx <= end_frame_idx; idx++)
            {
                tmp_scans.push_back(scans[idx]);
                tmp_scan_times_s.push_back(scan_times_s[idx]);
                tmp_poses.push_back(poses[idx]);
                tmp_frames_id.push_back(frames_id[idx]);
                if(i_use_deskewing == true)
                    tmp_scan_point_times_s.push_back(scan_point_times_s[idx]);
            }
            scans = tmp_scans;
            scan_times_s = tmp_scan_times_s;
            poses = tmp_poses;
            frames_id = tmp_frames_id;
            if(i_use_deskewing == true)
                scan_point_times_s = tmp_scan_point_times_s;
            std::cout << "Limit frames. scans size: " << scans.size() << ", poses size: " << poses.size() << ", times size: " << scan_times_s.size() << "\n";
        }
    }

    // Select files to use
    std::vector<Eigen::Affine3f> tmp_poses;
    std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> tmp_scans;
    std::vector<double> tmp_times_s;
    std::vector<int> tmp_frames_id;
    std::vector<std::vector<double>> tmp_scan_point_times_s;
    if(i_awv_mos.m_cfg_b_mapping_use_only_keyframes == true)
    {
        tmp_scans.push_back(scans[0]);
        tmp_poses.push_back(poses[0]);
        tmp_times_s.push_back(scan_times_s[0]);
        tmp_frames_id.push_back(frames_id[0]);
        if(i_use_deskewing == true)
            tmp_scan_point_times_s.push_back(scan_point_times_s[0]);

        for(int idx = 1; idx < poses.size(); idx++)
        {
            if(i_awv_mos.KeyframeSelection(tmp_poses.back(), tmp_times_s.back(), poses[idx], scan_times_s[idx]) == true)
            {
                tmp_poses.push_back(poses[idx]);
                tmp_scans.push_back(scans[idx]);
                tmp_times_s.push_back(scan_times_s[idx]);
                tmp_frames_id.push_back(frames_id[idx]);
                if(i_use_deskewing == true)
                    tmp_scan_point_times_s.push_back(scan_point_times_s[idx]);
            }
        }

        poses = tmp_poses;
        scans = tmp_scans;
        scan_times_s = tmp_times_s;
        frames_id = tmp_frames_id;
        if(i_use_deskewing == true)
            scan_point_times_s = tmp_scan_point_times_s;
        std::cout << "Mapping using keyframes. Select total " << poses.size() << " keyframes\n";
    }
    else
        std::cout << "Mapping using all frames. Select total " << poses.size() << " frames\n";

    if(i_use_deskewing == true)
    {
        int interp_scale = 100;
        int poses_interp_len = scan_times_s.size() * interp_scale;
        
        double step = (scan_times_s.back() - scan_times_s.front()) / (poses_interp_len - 1);

        std::vector<double> interped_pose_times(poses_interp_len);        
        for (int i = 0; i < poses_interp_len; ++i) 
            interped_pose_times[i] = scan_times_s.front() + i * step;

        std::vector<Eigen::Affine3f> interped_poses;
        FindInterpolatedPoses(interped_pose_times, bag_odom_times_s, bag_odom_poses, interped_poses);

        std::vector<Eigen::Affine3f> base_poses;
        std::vector<double> base_pose_times;
        std::vector<int> base_poses_index; 
        FindClosestPoses(scan_times_s, interped_pose_times, interped_poses, base_poses, base_pose_times, base_poses_index);

        for(int frame_idx = 0; frame_idx < scans.size() && ros::ok(); frame_idx++)
        {
            int curr_base_pose_idx = base_poses_index[frame_idx];
            int prev_base_pose_idx = -1;
            int next_base_pose_idx = -1;
            
            if(frame_idx == 0)
            {
                prev_base_pose_idx = base_poses_index[frame_idx];
                next_base_pose_idx = base_poses_index[frame_idx+2];
            }
            else if(frame_idx >= scans.size() - 1 - 2)
            {
                prev_base_pose_idx = base_poses_index[frame_idx-1];
                next_base_pose_idx = base_poses_index.back();
            }
            else
            {
                prev_base_pose_idx = base_poses_index[frame_idx-1];
                next_base_pose_idx = base_poses_index[frame_idx+2];
            }

            std::vector<double> pose_times_in_range;
            std::vector<Eigen::Affine3f> poses_in_range;
            for(int i = prev_base_pose_idx; i < next_base_pose_idx + 1; i++)
            {
                pose_times_in_range.push_back(interped_pose_times[i]);
                poses_in_range.push_back(interped_poses[i]);
            }

            pcl::PointCloud<PointTypeMOS>::Ptr curr_scan = scans[frame_idx];
            std::vector<double> curr_scan_point_times = scan_point_times_s[frame_idx];

            std::vector<Eigen::Affine3f> target_poses;
            std::vector<double> target_pose_times;
            std::vector<int> target_poses_index;
            FindClosestPoses(curr_scan_point_times, pose_times_in_range, poses_in_range, target_poses, target_pose_times, target_poses_index);
            
            Eigen::Affine3f tf_base_to_map = base_poses[frame_idx];
            Eigen::Affine3f tf_base_to_map_inv = tf_base_to_map.inverse();
            for(int po_i = 0; po_i < curr_scan->points.size(); po_i++)
            {
                Eigen::Affine3f tf_target_to_map = target_poses[po_i];
                Eigen::Affine3f tf_target_to_base = tf_base_to_map_inv * tf_target_to_map;

                Eigen::Vector3f point(curr_scan->points[po_i].x, curr_scan->points[po_i].y, curr_scan->points[po_i].z);
                Eigen::Vector3f deskewed_point = tf_target_to_base * point;
                curr_scan->points[po_i].x = deskewed_point.x();
                curr_scan->points[po_i].y = deskewed_point.y();
                curr_scan->points[po_i].z = deskewed_point.z();
            }

            float progress = static_cast<float>(frame_idx+1) / scans.size() * 100.0f;
            if((int)(progress * 1000) % 10 == 0)
                std::cout << "\rDeskeweing Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                        << progress << "% (" << (frame_idx+1) << "/" << scans.size() << ")" << std::flush;
        }
        std::cout << "\n";
    }

    o_scans = scans;
    o_poses = poses;
    o_times_s = scan_times_s;
    o_frames_id = frames_id;

    return true;
}

bool kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    
    ch = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    
    if(ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
    
    return false;
}

int main(int argc, char** argv)
{
    std::cout << "start\n";
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    ros::init(argc, argv, "run_static_mapping");

    AWV_MOS awv_mos;
    awv_mos.SaveConfigParams();

    ros::NodeHandle nh;
    bool use_bag_file, use_prediction_write, use_map_save, use_deskewing;
    std::string bag_file_path, scan_topic, odom_topic, prediction_write_path, map_save_path, scan_folder_path, label_folder_path, poses_file_path, times_file_path, calib_file_path;
    if (!nh.getParam("use_bag_file", use_bag_file)){ROS_WARN("Fail to get param - use_bag_file");}
    if (!nh.getParam("use_prediction_write", use_prediction_write)){ROS_WARN("Fail to get param - use_prediction_write");}
    if (!nh.getParam("prediction_write_path", prediction_write_path)){ROS_WARN("Fail to get param - prediction_write_path");}
    if (!nh.getParam("use_map_save", use_map_save)){ROS_WARN("Fail to get param - use_map_save");}
    if (!nh.getParam("map_save_path", map_save_path)){ROS_WARN("Fail to get param - map_save_path");}
    if(use_bag_file == true)
    {
        if (!nh.getParam("bag_file_path", bag_file_path)){ROS_WARN("Fail to get param - bag_file_path");}
        if (!nh.getParam("scan_topic", scan_topic)){ROS_WARN("Fail to get param - scan_topic");}
        if (!nh.getParam("odom_topic", odom_topic)){ROS_WARN("Fail to get param - odom_topic");}
        if (!nh.getParam("use_deskewing", use_deskewing)){ROS_WARN("Fail to get param - use_deskewing");}
        if (!nh.getParam("calib_file_path", calib_file_path)){ROS_WARN("Fail to get param - calib_file_path");}
    }
    else
    {
        if (!nh.getParam("scan_folder_path", scan_folder_path)){ROS_WARN("Fail to get param - scan_folder_path");}
        if (!nh.getParam("label_folder_path", label_folder_path)){ROS_WARN("Fail to get param - label_folder_path");}
        if (!nh.getParam("poses_file_path", poses_file_path)){ROS_WARN("Fail to get param - poses_file_path");}
        if (!nh.getParam("times_file_path", times_file_path)){ROS_WARN("Fail to get param - times_file_path");}
        if (!nh.getParam("calib_file_path", calib_file_path)){ROS_WARN("Fail to get param - calib_file_path");}
    }

    ros::Publisher pub_segmented_query_scan_all = nh.advertise<sensor_msgs::PointCloud2>(awv_mos.m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_all"), 100);
    ros::Publisher pub_segmented_query_scan_static = nh.advertise<sensor_msgs::PointCloud2>(awv_mos.m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_static"), 100);
    ros::Publisher pub_segmented_query_scan_dynamic = nh.advertise<sensor_msgs::PointCloud2>(awv_mos.m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_dynamic"), 100);
    ros::Publisher pub_static_map = nh.advertise<sensor_msgs::PointCloud2>(awv_mos.m_cfg_s_output_pc_namespace + std::string("/static_map"), 100);
    ros::Publisher pub_dynamic_map = nh.advertise<sensor_msgs::PointCloud2>(awv_mos.m_cfg_s_output_pc_namespace + std::string("/dynamic_map"), 100);
    tf::TransformBroadcaster br;

    // Read scan files
    std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> scans;
    std::vector<Eigen::Affine3f> poses; 
    std::vector<int> frames_id;
    std::vector<double> times_s;
    if(use_bag_file == true)
    {
        if(ReadDatasFromBag(awv_mos,
                            calib_file_path,
                            bag_file_path, 
                            scan_topic, 
                            odom_topic,
                            use_deskewing, 
                            scans,
                            poses,
                            times_s,
                            frames_id) == false){return -1;}        
    }
    else
    {
        if(ReadDatasFromKittiFormat(awv_mos, 
                                    calib_file_path, 
                                    poses_file_path, 
                                    scan_folder_path, 
                                    label_folder_path, 
                                    times_file_path,
                                    scans,
                                    poses,
                                    times_s,
                                    frames_id) == false){return -1;}
    }

    // Run static mapping
    pcl::PointCloud<PointTypeMOS>::Ptr static_map, dynamic_map;
    awv_mos.RunStaticMapping(scans, poses, frames_id, times_s, static_map, dynamic_map);

    // Save map
    if(awv_mos.m_cfg_b_mapping_use_save_map == true)
    {
        if (!std::filesystem::exists(map_save_path)) 
        {
            std::filesystem::create_directories(map_save_path);
            std::cout << "Create prediction map save path folder: " << map_save_path << std::endl;
        }

        size_t last_slash_idx;
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S_");

        std::string static_map_file_name = ss.str() + "static_map.pcd";
        std::string static_map_file_path = map_save_path + static_map_file_name;

        std::string total_map_file_name = ss.str() + "total_map.pcd";
        std::string total_map_file_path = map_save_path + total_map_file_name;

        pcl::io::savePCDFileBinary(static_map_file_path, *static_map);

        pcl::PointCloud<PointTypeMOS>::Ptr total_map(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
        *total_map = *static_map + *dynamic_map;
        pcl::io::savePCDFileBinary(total_map_file_path, *total_map);
        std::cout << "Save map to pcd file - Done. path: " << static_map_file_path << "\n";
    }

    // Write predictions
    if(awv_mos.m_cfg_b_use_prediction_write == true)
    {
        if (!std::filesystem::exists(prediction_write_path)) 
        {
            std::filesystem::create_directories(prediction_write_path);
            std::cout << "Create prediction write path folder: " << prediction_write_path << std::endl;
        }
        
        int idx = -1;
        while (ros::ok() && idx < (int)scans.size()-1) 
        {
            idx++;
            std::ostringstream oss;
            oss << std::setw(6) << std::setfill('0') << frames_id[idx] << ".label";
            std::string prediction_file_name = oss.str();
            std::string prediction_write_file_name = prediction_write_path + prediction_file_name;

            awv_mos.WritePrediction(scans[idx], prediction_write_file_name);

            float progress = static_cast<float>(idx + 1) / scans.size() * 100.0f;
            std::cout << "\rWriting predictions Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                    << progress << "% (frame idx: " << (idx) << "/" << scans.size()-1 << ")" << std::flush;
        }
        std::cout << "\n";
    }

    // Visualization
    if(awv_mos.m_cfg_b_mapping_use_visualization == true)
    {
        if(awv_mos.m_cfg_b_mapping_use_save_map == true)
        {
            if(static_map->points.size() > 1e7)
            {
                std::cout << "Num of map points are too many to visualize.\n";
                std::cout << "Down sampling - Start. map size: " << static_map->points.size() << "\n";
                float leaf_size = awv_mos.m_cfg_f_mapping_voxel_leaf_size_m;
                pcl::VoxelGrid<PointTypeMOS> voxel_grid_filter;
                while(static_map->points.size() > 1e7)
                {
                    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
                    voxel_grid_filter.setInputCloud(static_map);
                    voxel_grid_filter.filter(*static_map);
                    leaf_size = leaf_size * 1.5;
                }
                std::cout << "Down sampling - Done. map size: " << static_map->points.size() << "\n";
            }

            sensor_msgs::PointCloud2 static_map_msg;
            pcl::toROSMsg(*static_map, static_map_msg);
            static_map_msg.header.stamp = ros::Time::now();;
            static_map_msg.header.frame_id = "map";
            pub_static_map.publish(static_map_msg);

            sensor_msgs::PointCloud2 dynamic_map_msg;
            pcl::toROSMsg(*dynamic_map, dynamic_map_msg);
            dynamic_map_msg.header.stamp = ros::Time::now();;
            dynamic_map_msg.header.frame_id = "map";
            pub_dynamic_map.publish(dynamic_map_msg);
        }

        std::cout << "Visualization start. Press: [space] to pause/resume, [b] for prev step, [n] for step\n";
        bool first_publish = false;
        bool pause_publish = true;
        bool step_forward = false;
        bool step_backward = false;
        int idx = -1;
        float time_per_frame = (times_s.back() - times_s.front()) / times_s.size();
        while (ros::ok() && idx < (int)scans.size()-1) 
        {
            if (kbhit()) {
                char c = getchar();
                if (c == ' ') 
                    pause_publish = !pause_publish;
                else if (c == 'n' && pause_publish)
                    step_forward = true;
                else if (c == 'b' && pause_publish)
                {
                    step_backward = true;
                    idx -= 2;
                    if(idx < -1)
                        idx = -1;
                }
            }

            if (!first_publish || !pause_publish || step_forward || step_backward)
            {
                idx++;
                ros::Time pres_stamp = ros::Time::now();
                float tmp_x, tmp_y, tmp_z, tmp_roll, tmp_pitch, tmp_yaw;
                pcl::getTranslationAndEulerAngles (poses[idx], tmp_x, tmp_y, tmp_z, tmp_roll, tmp_pitch, tmp_yaw);
                auto q = tf::createQuaternionMsgFromRollPitchYaw(tmp_roll, tmp_pitch, tmp_yaw);

                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = pres_stamp;
                transformStamped.header.frame_id = "map";
                transformStamped.child_frame_id = "lidar_link";;
                transformStamped.transform.translation.x = poses[idx].translation().x();
                transformStamped.transform.translation.y = poses[idx].translation().y();
                transformStamped.transform.translation.z = poses[idx].translation().z();
                transformStamped.transform.rotation.x = q.x;
                transformStamped.transform.rotation.y = q.y;
                transformStamped.transform.rotation.z = q.z;
                transformStamped.transform.rotation.w = q.w;
                br.sendTransform(transformStamped);

                pcl::PointCloud<PointTypeMOS>::Ptr segmented_query_scan_all(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                pcl::copyPointCloud(*scans[idx], *segmented_query_scan_all);

                sensor_msgs::PointCloud2 segmented_query_scan_all_msg;
                pcl::toROSMsg(*segmented_query_scan_all, segmented_query_scan_all_msg);
                segmented_query_scan_all_msg.header.stamp = pres_stamp;
                segmented_query_scan_all_msg.header.frame_id = "lidar_link";
                pub_segmented_query_scan_all.publish(segmented_query_scan_all_msg);

                pcl::PointCloud<PointTypeMOS>::Ptr segmented_query_scan_static(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                pcl::PointCloud<PointTypeMOS>::Ptr segmented_query_scan_dynamic(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                for (const auto& point : segmented_query_scan_all->points) 
                {
                    if(point.r >= point.g || point.b >= point.g) // Filtering non-dynamic points
                        segmented_query_scan_static->points.push_back(point);
                    else
                        segmented_query_scan_dynamic->points.push_back(point);
                }
                sensor_msgs::PointCloud2 segmented_query_scan_static_msg;
                pcl::toROSMsg(*segmented_query_scan_static, segmented_query_scan_static_msg);
                segmented_query_scan_static_msg.header.stamp = pres_stamp;
                segmented_query_scan_static_msg.header.frame_id = "lidar_link";
                pub_segmented_query_scan_static.publish(segmented_query_scan_static_msg);
                sensor_msgs::PointCloud2 segmented_query_scan_dynamic_msg;
                pcl::toROSMsg(*segmented_query_scan_dynamic, segmented_query_scan_dynamic_msg);
                segmented_query_scan_dynamic_msg.header.stamp = pres_stamp;
                segmented_query_scan_dynamic_msg.header.frame_id = "lidar_link";
                pub_segmented_query_scan_dynamic.publish(segmented_query_scan_dynamic_msg);

                float progress = static_cast<float>(idx + 1) / scans.size() * 100.0f;
                std::cout << "\rMOS scan visualization(speed x5) Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                        << progress << "% (frame idx: " << (idx) << "/" << scans.size()-1 << ")" << std::flush;

                step_forward = false;
                step_backward = false;
                first_publish = true;
            }
            ros::Duration(time_per_frame/5).sleep();
        }
    }
    std::cout << "\n";

    return 0;
}