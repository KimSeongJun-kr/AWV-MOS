#define PCL_NO_PRECOMPILE 

#include "awv_mos_lio/cloud_info.h"

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <opencv2/opencv.hpp>

#include <tbb/parallel_for.h>
#include <tbb/global_control.h>

#ifdef USE_EVALUATION_POINT_TYPE
    struct EIGEN_ALIGN16 PointXYZIRTRGBL
    {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        std::uint16_t ring;
        float time;
        PCL_ADD_RGB
        std::uint16_t label;
        PCL_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
    } ;                    // enforce SSE padding for correct memory alignment

    POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTRGBL,
                                    (float, x, x) (float, y, y) (float, z, z) 
                                    (float, intensity, intensity)
                                    (std::uint16_t, ring, ring)
                                    (float, time, time)
                                    (std::uint32_t, rgb, rgb)
                                    (std::uint16_t, label, label))

    // Evaluation Expression
    using PointTypeFE = PointXYZIRTRGBL;
    using PointType = PointXYZIRTRGBL;
    using PointTypeMOS = PointXYZIRTRGBL;
#else
    struct PointXYZIRT
    {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY;
        std::uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
    POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
        (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
        (std::uint16_t, ring, ring) (float, time, time)
    )

    struct PointXYZIRGB
    {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        PCL_ADD_RGB
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
    POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRGB,
        (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
        (std::uint32_t, rgb, rgb)
    )
    // Normal Expression
    using PointTypeFE = PointXYZIRT;
    using PointType = pcl::PointXYZI;
    using PointTypeMOS = PointXYZIRGB;
#endif

const int queueLength = 2000;

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct RangePhiIdx{
    float range_m;
    float phi_deg;
    float time_s;
    int idx;

    RangePhiIdx(float i_range_m, 
                float i_phi_deg, 
                float i_time_s,
                int i_idx):
        range_m(i_range_m),
        phi_deg(i_phi_deg),
        time_s(i_time_s),
        idx(i_idx){}
};

enum class SensorType { VELODYNE, OUSTER, LIVOX, RSLIDAR };

class FeatureExtractionModule
{
private:

    // Deskewing variables
    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;
    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;
    bool odom_init_flag;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointTypeFE>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointTypeMOS>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;
    double m_odom_time_s;

    awv_mos_lio::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    std::vector<int> columnIdnCountVec;

    double startOri;
    double endOri;


    // Feature extraction variables    
    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubFeaturePoints;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    // Variables for no pixel loss
    std::vector<std::vector<RangePhiIdx>> m_rings_range_phi_idx_vector_m;
    std::vector<std::vector<float>> m_rings_range_vector_m;
    std::vector<int> m_rings_start_index_vector;
    std::vector<int> m_rings_end_index_vector;
    std::vector<float> m_points_range_vector_m;
    std::vector<float> m_points_phi_vector_deg;
    std::vector<int> m_points_idx_vector;

    pcl::PointCloud<PointType*>::Ptr  m_reordered_scan;
    pcl::PointCloud<PointType>::Ptr   m_featured_scan;

    float ang_res_x;

public:
    ros::NodeHandle nh;

    // AWV_MOS config Params
    // - CPU Params
    int m_cfg_n_num_cpu_cores;

    // LIO-SAM Params
    // Topics
    std::string pointCloudTopic;
    std::string imuTopic;
    std::string odomTopic;

    // Frames
    std::string lidarFrame;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    float scan_rate;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;
    std::vector<float> m_cfg_vec_f_ring_angle_pattern;

    // IMU
    std::vector<double> extRotV;
    std::vector<double> extRPYV;
    std::vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;



    FeatureExtractionModule():
    deskewFlag(0)
    {
        // AWV-MOS param
        nh.param<int>("awv_mos/m_cfg_n_num_cpu_cores", m_cfg_n_num_cpu_cores, -1);
        nh.param<int>("awv_mos/m_cfg_n_num_cpu_cores", m_cfg_n_num_cpu_cores, -1);
        if(m_cfg_n_num_cpu_cores <= 0)
            m_cfg_n_num_cpu_cores = std::thread::hardware_concurrency();
        // LIO-SAM param
        nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        std::string sensorStr;
        nh.param<std::string>("lio_sam/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
            sensor = SensorType::VELODYNE;
        else if (sensorStr == "rslidar")
            sensor = SensorType::RSLIDAR;
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
            ros::shutdown();
        }
        nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<float>("lio_sam/scan_rate", scan_rate, 1);
        nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
        nh.param<std::vector<float>>("lio_sam/m_cfg_vec_f_ring_angle_pattern", m_cfg_vec_f_ring_angle_pattern, std::vector<float>());

        nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);
        nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<std::vector<double>>("lio_sam/extrinsicRot", extRotV, std::vector<double>());
        nh.param<std::vector<double>>("lio_sam/extrinsicRPY", extRPYV, std::vector<double>());
        nh.param<std::vector<double>>("lio_sam/extrinsicTrans", extTransV, std::vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &FeatureExtractionModule::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &FeatureExtractionModule::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 100, &FeatureExtractionModule::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserCloudInfo = nh.advertise<awv_mos_lio::cloud_info> ("lio_sam/feature/cloud_info", 1);
        pubFeaturePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_feature", 1);

        ang_res_x = 360.0/float(Horizon_SCAN);
        odom_init_flag = false;

        allocateMemory();
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

/* Deskewing functionse */
    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointTypeFE>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointTypeMOS>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);
        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        m_reordered_scan.reset(new pcl::PointCloud<PointType*>());
        m_featured_scan.reset(new pcl::PointCloud<PointType>());

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();

        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);

        startOri = 0;
        endOri = 0;
        
        m_featured_scan->clear();
        m_reordered_scan->clear();
        m_rings_range_phi_idx_vector_m.clear();
        m_rings_range_phi_idx_vector_m.reserve(N_SCAN);
        for(int i = 0; i < N_SCAN; i++)
        {
            std::vector<RangePhiIdx> ring_range_phi_idx_vec;
            ring_range_phi_idx_vec.reserve(Horizon_SCAN*1.1);
            m_rings_range_phi_idx_vector_m.push_back(ring_range_phi_idx_vec);
        }
        m_rings_start_index_vector.assign(N_SCAN, 0);;
        m_rings_end_index_vector.assign(N_SCAN, 0);;
    }

    ~FeatureExtractionModule(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
        odom_init_flag = true;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        static tbb::global_control global_limit(tbb::global_control::max_allowed_parallelism, m_cfg_n_num_cpu_cores);
        
        // Runtime check variables
        static std::chrono::duration<double, std::milli> duration;
        static std::chrono::high_resolution_clock clock;
        static std::chrono::time_point<std::chrono::high_resolution_clock> total_start_time;
        static std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
        static int tmp_count = 0;
        tmp_count++;
        double total_time_ms = 0;
        static double sum_total_time_ms = 0;
        double cashe_computing_time_s = 0;
        static double sum_cashe_computing_time_s = 0;
        double IP_total_computing_time_s = 0;
        static double sum_IP_total_computing_time_s = 0;
        double FE_total_computing_time_s = 0;
        static double sum_FE_total_computing_time_s = 0;

        /* Deskewing part */
        start_time = clock.now();

        cachePointCloud(laserCloudMsg);

        duration = std::chrono::high_resolution_clock::now() - start_time;
        cashe_computing_time_s = duration.count();
        sum_cashe_computing_time_s += cashe_computing_time_s;

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur)
        {
            ROS_WARN("LiDAR msg older than IMU msg");
            return;
        }

        while(imuQueue.back().header.stamp.toSec() < timeScanEnd && ros::ok())
        {
            ROS_DEBUG("Waiting for IMU data ...");
            ros::Duration(0.005).sleep();  
            continue;
        }

        while(odom_init_flag == true && odomQueue.back().header.stamp.toSec() < timeScanEnd && ros::ok())
        {
            ROS_DEBUG("Waiting for Odom data ...");
            ros::Duration(0.005).sleep();  
            continue;
        }

        total_start_time = clock.now();

        deskewInfo();
        projectPointCloud();
        cloudExtraction();

        duration = std::chrono::high_resolution_clock::now() - total_start_time;
        IP_total_computing_time_s = duration.count();
        sum_IP_total_computing_time_s += IP_total_computing_time_s;

        /* Feature extraction part */
        total_start_time = clock.now();

        calculateSmoothness();
        markOccludedPoints();
        extractFeatures();
        publishFeatureCloud();
        resetParameters();

        duration = std::chrono::high_resolution_clock::now() - total_start_time;
        FE_total_computing_time_s = duration.count();
        sum_FE_total_computing_time_s += IP_total_computing_time_s;

        static double sum_total_computing_time_s = 0;
        sum_total_computing_time_s += cashe_computing_time_s + IP_total_computing_time_s + FE_total_computing_time_s;
        static int frame_count = 0;
        frame_count++;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << frame_count - 1 
                    << " [FE Module Ave Time] Total: " << sum_total_computing_time_s / frame_count
                    << ", Ca: " << sum_cashe_computing_time_s / frame_count
                    << ", IP: " << sum_IP_total_computing_time_s / frame_count 
                    << ", FE: " << sum_FE_total_computing_time_s / frame_count << "\n"; 
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (sensor == SensorType::VELODYNE)
        {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

            // get timestamp
            cloudHeader = laserCloudMsg->header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanEnd = timeScanCur + 1.0 / scan_rate;
        }
        else if (sensor == SensorType::RSLIDAR)
        {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

            // get timestamp
            cloudHeader = laserCloudMsg->header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanEnd = timeScanCur + 1.0 / scan_rate;

            tbb::parallel_for
            (
                size_t(0), (size_t) laserCloudIn->points.size(), [&](size_t idx) 
                {
                    if(std::isnan(laserCloudIn->points[idx].x) == false)
                    {
                        int column = idx % Horizon_SCAN;
                        laserCloudIn->points[idx].time = 0.1 * (double) column / (double) Horizon_SCAN;

                        // int ring = idx / Horizon_SCAN;
                        // laserCloudIn->points[idx].ring = ring;

                        float point_vertical_Angle_deg = atan2(laserCloudIn->points[idx].z, sqrt(laserCloudIn->points[idx].x * laserCloudIn->points[idx].x + laserCloudIn->points[idx].y * laserCloudIn->points[idx].y)) * 180 / M_PI;
                        for(int ring_idx = 0; ring_idx < (int)m_cfg_vec_f_ring_angle_pattern.size() - 1; ring_idx++)
                        {
                            float front_ring_angle_deg = m_cfg_vec_f_ring_angle_pattern[ring_idx];
                            float back_ring_angle_deg = m_cfg_vec_f_ring_angle_pattern[ring_idx + 1];
                            float angle_diff_front = point_vertical_Angle_deg - front_ring_angle_deg;
                            float angle_diff_back = point_vertical_Angle_deg - back_ring_angle_deg;

                            if(ring_idx == 0 && point_vertical_Angle_deg <= front_ring_angle_deg)
                            {
                                laserCloudIn->points[idx].ring = ring_idx;
                                break;
                            }
                            else if(ring_idx == (int)m_cfg_vec_f_ring_angle_pattern.size() - 1 && point_vertical_Angle_deg >= back_ring_angle_deg)
                            {
                                laserCloudIn->points[idx].ring = ring_idx;
                                break;
                            }
                            else
                            {
                                if(fabs(angle_diff_front) <= 1e-2)
                                {
                                    laserCloudIn->points[idx].ring = ring_idx;
                                    break;
                                }
                                else if(fabs(angle_diff_front) + fabs(angle_diff_back) > fabs(angle_diff_front + angle_diff_back))
                                {
                                    if(fabs(angle_diff_front) < fabs(angle_diff_back))
                                    {
                                        laserCloudIn->points[idx].ring = ring_idx;
                                    }
                                    else
                                    {
                                        laserCloudIn->points[idx].ring = ring_idx + 1;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }       
            );     
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }
        
        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            std::vector<int> indices;
            pcl::PointCloud<PointTypeFE>::Ptr copy(new pcl::PointCloud<PointTypeFE>);
            *copy = *laserCloudIn;
            pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        }

        if(sensor != SensorType::RSLIDAR)
        {
            // check ring channel
            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)laserCloudMsg->fields.size(); ++i)
                {
                    if (laserCloudMsg->fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
                else
                {
                    ringFlag = 1;
                }
            }

            // check point time
            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (auto &field : laserCloudMsg->fields)
                {
                    if (field.name == "time" || field.name == "t")
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1)
                    ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }
        }
        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.05)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.05)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
        {
            return;
        }

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
        {
            return;
        }
        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (startOdomMsg.header.stamp.toSec() < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (endOdomMsg.header.stamp.toSec() < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        m_odom_time_s = endOdomMsg.header.stamp.toSec();
        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.
        if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
            return;

        float ratio = relTime / (m_odom_time_s - timeScanCur);

        *posXCur = ratio * odomIncreX;
        *posYCur = ratio * odomIncreY;
        *posZCur = ratio * odomIncreZ;
    }

    PointTypeFE deskewPoint(PointTypeFE *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            float first_rotXCur, first_rotYCur, first_rotZCur;
            findRotation(timeScanCur, &first_rotXCur, &first_rotYCur, &first_rotZCur);

            float first_posXCur, first_posYCur, first_posZCur;
            findPosition(0.0, &first_posXCur, &first_posYCur, &first_posZCur);
            transStartInverse = (pcl::getTransformation(first_posXCur, first_posYCur, first_posZCur, first_rotXCur, first_rotYCur, first_rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointTypeFE newPoint;
        newPoint = *point;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);

        return newPoint;
    }

    void projectPointCloud()
    {
        // range image projection
        int cloudSize = laserCloudIn->points.size();

        tbb::parallel_for
        (
            size_t(0), (size_t) cloudSize, [&](size_t i) 
            {
                PointTypeFE thisPoint(laserCloudIn->points[i]);

                float range = sqrt(thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y + thisPoint.z*thisPoint.z);
                int rowIdn = thisPoint.ring;

                if (range < lidarMinRange || range > lidarMaxRange ||
                rowIdn < 0 || rowIdn >= N_SCAN ||
                rowIdn % downsampleRate != 0)
                    return;

                int columnIdn = -1;
                if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER || sensor == SensorType::RSLIDAR)
                {
                    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                    if (columnIdn >= Horizon_SCAN)
                        columnIdn -= Horizon_SCAN;
                }
                
                if (columnIdn < 0 || columnIdn >= Horizon_SCAN ||
                    rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                    return;

                thisPoint = deskewPoint(&thisPoint, thisPoint.time);

                int index = columnIdn + rowIdn * Horizon_SCAN;

                rangeMat.at<float>(rowIdn, columnIdn) = range;

                PointType thisPoint2;
                pcl::copyPoint(thisPoint, thisPoint2);
                fullCloud->points[index] = thisPoint2;
            }
        );
        return;
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            if(count == 0)
                cloudInfo.startRingIndex[i] = 5;
            else
                cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    PointTypeMOS point;
                    pcl::copyPoint(fullCloud->points[j + i*Horizon_SCAN], point);
                    extractedCloud->push_back(point);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }
    
/* Feature extraciton functions */
    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
            // cloudSmoothness for sorting
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

            if (columnDiff < 10){
                // 10 pixel diff in range image
                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            // parallel beam
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures()
    {

        tbb::parallel_for
        (
            size_t(0), (size_t) 6 * N_SCAN, [&](size_t idx) 
            {
                int i = idx / 6;
                int j = idx % 6;

                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    return;

                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep+1, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20){
                            cloudLabel[ind] = 1;
                            extractedCloud->points[ind].intensity = 1;
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){
                        extractedCloud->points[k].intensity = 0;
                    }
                }
            }
        );
    }

    void freeCloudInfoMemory()
    {
        std::fill(cloudInfo.startRingIndex.begin(), cloudInfo.startRingIndex.end(), 0);
        std::fill(cloudInfo.endRingIndex.begin(), cloudInfo.endRingIndex.end(), 0);
        std::fill(cloudInfo.pointColInd.begin(), cloudInfo.pointColInd.end(), 0);
        std::fill(cloudInfo.pointRange.begin(), cloudInfo.pointRange.end(), 0);
    }

    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_feature  = publishCloud(pubFeaturePoints,  extractedCloud,  cloudHeader.stamp, lidarFrame);

        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }

    template<typename T>
    void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
    {
        *angular_x = thisImuMsg->angular_velocity.x;
        *angular_y = thisImuMsg->angular_velocity.y;
        *angular_z = thisImuMsg->angular_velocity.z;
    }

    template<typename T>
    void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
    {
        double imuRoll, imuPitch, imuYaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

        *rosRoll = imuRoll;
        *rosPitch = imuPitch;
        *rosYaw = imuYaw;
    }

    template<typename T>
    sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        if (thisPub.getNumSubscribers() != 0)
            thisPub.publish(tempCloud);
        return tempCloud;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtractionModule FEM;
    
    ROS_INFO("\033[1;32m----> Feature Extraction Module Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
