#include "AWV_MOS.h"
#include <filesystem>
#include <queue>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

std::queue<sensor_msgs::PointCloud2> scan_msg_queue;
std::queue<nav_msgs::Odometry> odom_msg_queue;
std::mutex mtx;

void ScanCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    scan_msg_queue.push(*msg);
    return;
}

void OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    odom_msg_queue.push(*msg);
    return;
}

bool ReadSceneTime(const std::string &i_filename, std::vector<double> &o_start_timestamps, std::vector<std::string> &o_scene_names) {
    std::ifstream file(i_filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file" << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string index, scene_name_label, scene_name, start_stamp_label, start_stamp, end_stamp_label, end_stamp;

        if (!(ss >> index >> scene_name_label >> scene_name >> start_stamp_label >> start_stamp >> end_stamp_label >> end_stamp)) {
            continue; // Skip any malformed lines
        }

        try {
            double start_timestamp = std::stod(start_stamp);
            o_start_timestamps.push_back(start_timestamp);
            o_scene_names.push_back(scene_name);
        } catch (const std::invalid_argument &e) {
            std::cerr << "Invalid number in file: " << e.what() << std::endl;
        }
    }

    file.close();
    return true;
}

int findClosestIndex(const std::vector<double>& vec, double value) {
    if (vec.empty()) {
        throw std::invalid_argument("The vector is empty.");
    }

    auto closest = std::min_element(vec.begin(), vec.end(), [value](double a, double b) {
        return std::abs(a - value) < std::abs(b - value);
    });

    return std::distance(vec.begin(), closest);
}

int main(int argc, char** argv)
{
    std::cout << "start\n";
    ros::init(argc, argv, "run_online_mos");
    ros::NodeHandle nh;

    AWV_MOS awv_mos;
    awv_mos.SaveConfigParams();
    
    bool use_prediction_write;
    std::string prediction_write_path, scan_topic, odom_topic;
    if (!nh.getParam("use_prediction_write", use_prediction_write)){ROS_WARN("Fail to get param - use_prediction_write");}
    if (!nh.getParam("prediction_write_path", prediction_write_path)){ROS_WARN("Fail to get param - prediction_write_path");}
    if (!nh.getParam("scan_topic", scan_topic)){ROS_WARN("Fail to get param - scan_topic");}
    if (!nh.getParam("odom_topic", odom_topic)){ROS_WARN("Fail to get param - odom_topic");}

    ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(scan_topic, 10000, ScanCallback);
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 10000, OdomCallback);

    ros::Publisher pub_segmented_query_scan_all = nh.advertise<sensor_msgs::PointCloud2>(awv_mos.m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_all"), 100);
    ros::Publisher pub_segmented_query_scan_static = nh.advertise<sensor_msgs::PointCloud2>(awv_mos.m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_static"), 100);
    ros::Publisher pub_segmented_query_scan_dynamic = nh.advertise<sensor_msgs::PointCloud2>(awv_mos.m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_dynamic"), 100);

    tf::TransformBroadcaster br;

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    pcl::PointCloud<PointTypeMOS>::Ptr scan(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());

    pcl::PointCloud<PointTypeMOS>::Ptr segmented_scan;

    double prev_scan_time_s = 0;

    // Runtime check variables
    static std::chrono::duration<double, std::milli> duration;
    static std::chrono::high_resolution_clock clock;
    static std::chrono::time_point<std::chrono::high_resolution_clock> total_start_time;
    static std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    static int tmp_count = 0;
    tmp_count++;
    static double sum_total_time_ms = 0;
    double total_time_ms = 0;

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        std::lock_guard<std::mutex> lock(mtx);
        while (!scan_msg_queue.empty() && !odom_msg_queue.empty()) {
            sensor_msgs::PointCloud2 scan_msg = scan_msg_queue.front();
            nav_msgs::Odometry odom_msg = odom_msg_queue.front();

            double scan_time_s = scan_msg.header.stamp.toSec();
            double odom_time_s = odom_msg.header.stamp.toSec();
            double time_diff_s = scan_time_s - odom_time_s;

            if (fabs(time_diff_s) <= 0.025) 
            {
                Eigen::Affine3f tf_frame_to_map = Eigen::Affine3f::Identity();
                tf_frame_to_map.translation() << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
                Eigen::Quaternionf quat(odom_msg.pose.pose.orientation.w, 
                                        odom_msg.pose.pose.orientation.x, 
                                        odom_msg.pose.pose.orientation.y, 
                                        odom_msg.pose.pose.orientation.z);
                tf_frame_to_map.rotate(quat);

                pcl::fromROSMsg(scan_msg, *scan);
                
                total_start_time = clock.now();

                double time_s = scan_msg.header.stamp.toSec();
                bool is_keyframe = awv_mos.KeyframeSelection(tf_frame_to_map, time_s);
                static int frame_count = 0;
                awv_mos.RunOnlineMOS(scan, tf_frame_to_map, ++frame_count-1, time_s, is_keyframe);
                awv_mos.GetSegmentedScan(segmented_scan);

                scan_msg_queue.pop();
                odom_msg_queue.pop();

                duration = std::chrono::high_resolution_clock::now() - total_start_time;
                total_time_ms = duration.count();
                sum_total_time_ms += total_time_ms;

                std::cout << "\r" << frame_count - 1 << " [MOS Module Ave Time] Total average time ms: " << std::setw(4) << std::fixed << std::setprecision(1)
                        << sum_total_time_ms / frame_count << " ms" << std::flush;
                        
                if(awv_mos.m_cfg_b_use_prediction_write == true)
                {
                    if (!std::filesystem::exists(prediction_write_path)) 
                    {
                        std::filesystem::create_directories(prediction_write_path);
                        std::cout << "Create prediction write path folder: " << prediction_write_path << std::endl;
                    }

                    std::ostringstream oss;
                    oss << std::setw(6) << std::setfill('0') << frame_count-1 << ".label";
                    std::string prediction_file_name = oss.str();
                    std::string prediction_write_file_name = prediction_write_path + prediction_file_name;
                    awv_mos.WritePrediction(segmented_scan, prediction_write_file_name);
                }
                if(awv_mos.m_cfg_b_publish_pc == true)
                {
                    float tmp_x, tmp_y, tmp_z, tmp_roll, tmp_pitch, tmp_yaw;
                    pcl::getTranslationAndEulerAngles (tf_frame_to_map, tmp_x, tmp_y, tmp_z, tmp_roll, tmp_pitch, tmp_yaw);
                    auto q = tf::createQuaternionMsgFromRollPitchYaw(tmp_roll, tmp_pitch, tmp_yaw);

                    geometry_msgs::TransformStamped transformStamped;
                    transformStamped.header.stamp = odom_msg.header.stamp;
                    transformStamped.header.frame_id = "map";
                    transformStamped.child_frame_id = scan_msg.header.frame_id;
                    transformStamped.transform.translation.x = tf_frame_to_map.translation().x();
                    transformStamped.transform.translation.y = tf_frame_to_map.translation().y();
                    transformStamped.transform.translation.z = tf_frame_to_map.translation().z();
                    transformStamped.transform.rotation.x = q.x;
                    transformStamped.transform.rotation.y = q.y;
                    transformStamped.transform.rotation.z = q.z;
                    transformStamped.transform.rotation.w = q.w;
                    br.sendTransform(transformStamped);

                    sensor_msgs::PointCloud2 segmented_query_scan_all_all;
                    pcl::toROSMsg(*segmented_scan, segmented_query_scan_all_all);
                    segmented_query_scan_all_all.header = scan_msg.header;
                    pub_segmented_query_scan_all.publish(segmented_query_scan_all_all);

                    pcl::PointCloud<PointTypeMOS>::Ptr segmented_query_scan_static(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                    pcl::PointCloud<PointTypeMOS>::Ptr segmented_query_scan_dynamic(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                    for (const auto& point : segmented_scan->points) 
                    {
                        if(point.r >= point.g || point.b >= point.g) // Filtering non-dynamic points
                            segmented_query_scan_static->points.push_back(point);
                        else
                            segmented_query_scan_dynamic->points.push_back(point);
                    }
                    sensor_msgs::PointCloud2 segmented_query_scan_static_msg, segmented_query_scan_dynamic_msg;
                    pcl::toROSMsg(*segmented_query_scan_static, segmented_query_scan_static_msg);
                    pcl::toROSMsg(*segmented_query_scan_dynamic, segmented_query_scan_dynamic_msg);
                    segmented_query_scan_static_msg.header = scan_msg.header;
                    segmented_query_scan_dynamic_msg.header = scan_msg.header;
                    pub_segmented_query_scan_static.publish(segmented_query_scan_static_msg);
                    pub_segmented_query_scan_dynamic.publish(segmented_query_scan_dynamic_msg);
                }
            } 
            else 
            {
                if (scan_time_s < odom_time_s) 
                {
                    ROS_WARN("older scan data: %f", time_diff_s);
                    scan_msg_queue.pop();
                } 
                else 
                {
                    ROS_WARN("older odom data: %f", time_diff_s);
                    odom_msg_queue.pop();
                }
            }
        }
    }

    

    return 0;
}