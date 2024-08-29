#define PCL_NO_PRECOMPILE 

#include "awv_mos_lio/cloud_info.h"

#include <ros/ros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

// Include for MOS
#include <AWV_MOS.hpp>
#include <filesystem>

#include <tbb/parallel_for.h>
#include <tbb/global_control.h>

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

// Evaluation Expression
using PointTypePose = PointXYZIRPYT;

#ifndef _POINT_TYPE_DEFINITION_
#define _POINT_TYPE_DEFINITION_
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
    using PointType = PointXYZIRTRGBL;
    using PointTypeMOS = PointXYZIRTRGBL;
#else
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
    using PointType = pcl::PointXYZI;
    using PointTypeMOS = PointXYZIRGB;
#endif
#endif


using namespace gtsam;

static std::mutex m_mutexInputPointCloud;

class mapOptimization
{
public:
    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubLaserOdometryInitial;
    ros::Publisher pubKeyPoses;
    ros::Publisher m_pubSrroundKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;

    ros::Publisher pubSLAMInfo;

    ros::Subscriber subCloud;

    awv_mos_lio::cloud_info cloudInfo;
    std::deque<awv_mos_lio::cloud_info> m_vec_cloud_info_queue;

    std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> cornerCloudKeyFrames;
    std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> surfCloudKeyFrames;
    std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> CloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr m_surroundingKeyPoses;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    pcl::PointCloud<PointTypeMOS>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointTypeMOS>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointTypeMOS>::Ptr laserCloudCornerLastDS; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointTypeMOS>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization
    pcl::PointCloud<PointTypeMOS>::Ptr m_pres_scanframe;
    pcl::PointCloud<PointTypeMOS>::Ptr m_pres_initial_segmented_scan;    
    pcl::PointCloud<PointTypeMOS>::Ptr m_pres_segmented_scan;

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointTypeMOS>, pcl::PointCloud<PointTypeMOS>>> laserCloudMapContainer;
    pcl::PointCloud<PointTypeMOS>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointTypeMOS>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointTypeMOS>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointTypeMOS>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointTypeMOS>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointTypeMOS>::Ptr kdtreeSurfFromMap;


    pcl::VoxelGrid<PointTypeMOS> downSizeFilterCorner;
    pcl::VoxelGrid<PointTypeMOS> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];
    float transformincremental[6];
    float transformincrementalinitial[6];
    float InitialPose[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    int m_i_num_of_keyframes = 0;

    /* MOS variables */

    // MOS Class
    AWV_MOS m_awv_mos;

    // TF
    Eigen::Affine3f m_tf_scan_initial_to_map;
    Eigen::Affine3f m_tf_scan_increase_to_map;
    tf::TransformBroadcaster m_br;

    // Publisher
    ros::Publisher m_pub_segmented_query_scan_all;
    ros::Publisher m_pub_segmented_query_scan_static;
    ros::Publisher m_pub_segmented_query_scan_dynamic;
    int m_i_current_keyframe_index;
    int m_i_frame_count = 0;

    // time
    double m_d_pose_time_pres_s;

    // Pose
    std::vector<double> m_vec_pres_pose_m_deg;

    // loop closure time check
    bool m_b_loop_closure_done;

public:
    ros::NodeHandle nh;

    // AWV-MOS params
    std::string m_cfg_s_output_pc_namespace;
    float m_cfg_f_static_weight_ratio;
    bool m_cfg_b_use_prior_mos;
    bool m_cfg_b_publish_pc;
    int m_cfg_n_num_cpu_cores;

    // // LIO-SAM Params
    // //Frames
    std::string lidarFrame;
    std::string odometryFrame;
    std::string mapFrame;

    // Lidar Sensor Configuration
    int N_SCAN;
    int Horizon_SCAN;

    // IMU
    float imuRPYWeight;

    // LOAM
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // Surrounding map
    float surroundingKeyframeSearchRadius;
    int surroundingKeyframeMaxNum;

    mapOptimization()
    {
        // AWV-MOS params
        nh.param<std::string>("awv_mos/m_cfg_s_output_pc_namespace", m_cfg_s_output_pc_namespace, "/awv_mos/mos_pc");
        nh.param<float>("awv_mos/m_cfg_f_static_weight_ratio", m_cfg_f_static_weight_ratio, -1);
        nh.param<bool>("awv_mos/m_cfg_b_use_prior_mos", m_cfg_b_use_prior_mos, -1);
        nh.param<bool>("awv_mos/m_cfg_b_publish_pc", m_cfg_b_publish_pc, -1);
        nh.param<int>("awv_mos/m_cfg_n_num_cpu_cores", m_cfg_n_num_cpu_cores, -1);
        if(m_cfg_n_num_cpu_cores <= 0)
            m_cfg_n_num_cpu_cores = std::thread::hardware_concurrency();

        // LIO-SAM params
        nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");
        nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);
        nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);
        nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);
        nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);
        nh.param<int>("lio_sam/surroundingKeyframeMaxNum", surroundingKeyframeMaxNum, 10);

        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;

        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        m_pubSrroundKeyPoses        = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/surround_key_poses", 1);
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry_incremental", 1);
        pubLaserOdometryInitial      = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry_initial", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);

        subCloud = nh.subscribe<awv_mos_lio::cloud_info>("lio_sam/feature/cloud_info", 100, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

        pubSLAMInfo           = nh.advertise<awv_mos_lio::cloud_info>("lio_sam/mapping/slam_info", 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        allocateMemory();

        // m_evi_mos = new EviMOS();

        m_vec_pres_pose_m_deg.assign({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        // Publish
        m_pub_segmented_query_scan_all = nh.advertise<sensor_msgs::PointCloud2>(m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_all"), 1);
        m_pub_segmented_query_scan_static = nh.advertise<sensor_msgs::PointCloud2>(m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_static"), 1);
        m_pub_segmented_query_scan_dynamic = nh.advertise<sensor_msgs::PointCloud2>(m_cfg_s_output_pc_namespace + std::string("/segmented_query_scan_dynamic"), 1);

        m_awv_mos.SaveConfigParams();
    }

    ~mapOptimization()
    {}

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        m_surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());


        laserCloudCornerLast.reset(new pcl::PointCloud<PointTypeMOS>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointTypeMOS>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointTypeMOS>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointTypeMOS>()); // downsampled surf featuer set from odoOptimization
        m_pres_scanframe.reset(new pcl::PointCloud<PointTypeMOS>());

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointTypeMOS>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointTypeMOS>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointTypeMOS>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointTypeMOS>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointTypeMOS>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointTypeMOS>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const awv_mos_lio::cloud_infoConstPtr& msgIn)
    {
        m_mutexInputPointCloud.lock();
        m_vec_cloud_info_queue.push_front(*msgIn);
        m_mutexInputPointCloud.unlock();
        return;
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        *po = *pi;
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        return;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i] = pointFrom;
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        }
        return cloudOut;
    }
    
    pcl::PointCloud<PointTypeMOS>::Ptr transformPointCloudMOS(pcl::PointCloud<PointTypeMOS>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointTypeMOS>::Ptr cloudOut(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i] = pointFrom;
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

/* --------------------- Graph Optimization Functions */
    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization
        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transTobeIncre = trans2Affine3f(transformincremental);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                // Eigen::Affine3f transFinalIncre = transTobeIncre * transIncre;
                m_tf_scan_initial_to_map = transTobeIncre * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
                pcl::getTranslationAndEulerAngles(m_tf_scan_initial_to_map, transformincrementalinitial[3], transformincrementalinitial[4], transformincrementalinitial[5], 
                                                                    transformincrementalinitial[0], transformincrementalinitial[1], transformincrementalinitial[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }
        else
        {
            std::cout << "[updateInitialGuess] cloudInfo.odomAvailable = false\n";
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transTobeIncre = trans2Affine3f(transformincremental);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            // Eigen::Affine3f transFinalIncre = transTobeIncre * transIncre;
            m_tf_scan_initial_to_map = transTobeIncre * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            pcl::getTranslationAndEulerAngles(m_tf_scan_initial_to_map, transformincrementalinitial[3], transformincrementalinitial[4], transformincrementalinitial[5], 
                                                                transformincrementalinitial[0], transformincrementalinitial[1], transformincrementalinitial[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
        else
        {
            std::cout << "[updateInitialGuess] cloudInfo.imuAvailable == false\n";
        }

        return;
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 

        extractSequential();

        return;
    }

    void extractSequential()
    {
        // pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());

        static int prev_recent_idx = -1;
        if(prev_recent_idx != cloudKeyPoses3D->points.back().intensity)
        {
            m_surroundingKeyPoses->clear();
            m_surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points.back());
            PointType prev_pose = cloudKeyPoses3D->points.back();
            double keyframe_1d_odometry_m = 0;

            for(int idx = cloudKeyPoses3D->points.size() - 2; idx >= 0; idx--)
            {
                double odometry_m = sqrt((cloudKeyPoses3D->points[idx].x - prev_pose.x) * (cloudKeyPoses3D->points[idx].x - prev_pose.x)
                                        + (cloudKeyPoses3D->points[idx].y - prev_pose.y) * (cloudKeyPoses3D->points[idx].y - prev_pose.y)
                                        + (cloudKeyPoses3D->points[idx].z - prev_pose.z) * (cloudKeyPoses3D->points[idx].z - prev_pose.z));
                keyframe_1d_odometry_m += odometry_m;
                prev_pose = cloudKeyPoses3D->points[idx];

                if(keyframe_1d_odometry_m > surroundingKeyframeSearchRadius || m_surroundingKeyPoses->points.size() >= surroundingKeyframeMaxNum)
                    break;

                m_surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[idx]);
            }

            extractCloud(m_surroundingKeyPoses);
            
            prev_recent_idx = cloudKeyPoses3D->points.back().intensity;
        }

        return;
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointTypeMOS> laserCloudCornerTemp = *transformPointCloudMOS(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointTypeMOS> laserCloudSurfTemp = *transformPointCloudMOS(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();

        m_i_num_of_keyframes = cloudToExtract->points.size();

        return;
    }
    
    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();

        return;
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
        return;
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();
		
        tbb::parallel_for
        (
            size_t(0), (size_t) laserCloudCornerLastDSNum, [&](size_t i) 
            {                	
                PointType pointOri, pointSel;
                PointType coeff;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // pointOri = laserCloudCornerLastDS->points[i];
                pcl::copyPoint(laserCloudCornerLastDS->points[i], pointOri);
                
                pointAssociateToMap(&pointOri, &pointSel);

                PointTypeMOS pointSel2;
                pcl::copyPoint(pointSel, pointSel2);
                kdtreeCornerFromMap->nearestKSearch(pointSel2, 5, pointSearchInd, pointSearchSqDis);

                cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
                cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
                cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                        
                if (pointSearchSqDis[4] < 1.0) {
                    float cx = 0, cy = 0, cz = 0;
                    for (int j = 0; j < 5; j++) {
                        cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                        cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                        cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                    }
                    cx /= 5; cy /= 5;  cz /= 5;

                    float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                    float static_bel = 0;
                    for (int j = 0; j < 5; j++) {
                        float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                        float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                        float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                        a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                        a22 += ay * ay; a23 += ay * az;
                        a33 += az * az;

                        static_bel += laserCloudCornerFromMapDS->points[pointSearchInd[j]].r / 255.0;
                    }
                    a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;
                    static_bel /= 5;
                    float static_weight = 1.0 + static_bel * m_cfg_f_static_weight_ratio;
                    // float static_weight = 1.0 + static_bel;

                    matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                    matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                    matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                    cv::eigen(matA1, matD1, matV1);

                    if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                        float x0 = pointSel.x;
                        float y0 = pointSel.y;
                        float z0 = pointSel.z;
                        float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                        float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                        float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                        float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                        float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                        float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                        float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                        + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                        + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                        float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                        float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                        float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                        float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                        float ld2 = a012 / l12;

                        float s = 1 - 0.9 * fabs(ld2);

                        coeff.x = static_weight * s * la;
                        coeff.y = static_weight * s * lb;
                        coeff.z = static_weight * s * lc;
                        coeff.intensity = static_weight * s * ld2;

                        if (s > 0.1) {
                            laserCloudOriCornerVec[i] = pointOri;
                            coeffSelCornerVec[i] = coeff;
                            laserCloudOriCornerFlag[i] = true;
                        }
                    }
                }
            }
        );
        return;
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();


        tbb::parallel_for
        (
            size_t(0), (size_t) laserCloudSurfLastDSNum, [&](size_t i) 
            {       
                PointType pointOri, pointSel;
                PointType coeff;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // pointOri = laserCloudSurfLastDS->points[i];
                pcl::copyPoint(laserCloudSurfLastDS->points[i], pointOri);
                
                pointAssociateToMap(&pointOri, &pointSel); 
                PointTypeMOS pointSel2;
                pcl::copyPoint(pointSel, pointSel2);
                kdtreeSurfFromMap->nearestKSearch(pointSel2, 5, pointSearchInd, pointSearchSqDis);

                Eigen::Matrix<float, 5, 3> matA0;
                Eigen::Matrix<float, 5, 1> matB0;
                Eigen::Vector3f matX0;

                matA0.setZero();
                matB0.fill(-1);
                matX0.setZero();

                if (pointSearchSqDis[4] < 1.0) {
                    for (int j = 0; j < 5; j++) {
                        matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                        matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                        matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                    }

                    matX0 = matA0.colPivHouseholderQr().solve(matB0);

                    float pa = matX0(0, 0);
                    float pb = matX0(1, 0);
                    float pc = matX0(2, 0);
                    float pd = 1;

                    float ps = sqrt(pa * pa + pb * pb + pc * pc);
                    pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                    bool planeValid = true;
                    float static_bel = 0;
                    for (int j = 0; j < 5; j++) {
                        if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                                pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                                pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) { // TODO : Magic number? Same with lego loam Same with loam
                            planeValid = false;
                            break;
                        }
                        static_bel += laserCloudSurfFromMapDS->points[pointSearchInd[j]].r / 255.0;
                    }

                    static_bel /= 5;
                    float static_weight = 1.0 + static_bel * m_cfg_f_static_weight_ratio;
                    // float static_weight = 1.0 + static_bel;

                    if (planeValid) {
                        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                        float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                                + pointOri.y * pointOri.y + pointOri.z * pointOri.z));
                        // float s = 1 - 0.9 * fabs(pd2);

                        coeff.x = static_weight * s * pa;
                        coeff.y = static_weight * s * pb;
                        coeff.z = static_weight * s * pc;
                        coeff.intensity = static_weight * s * pd2;

                        if ( s > 0.1) {
                            laserCloudOriSurfVec[i] = pointOri;
                            coeffSelSurfVec[i] = coeff;
                            laserCloudOriSurfFlag[i] = true;
                        }
                    }
                }
            }
        );
        return;
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
        return;
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                static int divergence_count = 0;
                if (LMOptimization(iterCount) == true)
                {
                    // std::cout << "iterCount: " << iterCount << ", Divergence count: " << divergence_count << "\n";
                    break;
                }

                if(iterCount >= 29)
                {
                    divergence_count++;
                    std::cout << "\033[1;31m iterCount: " << iterCount << ", Divergence count: " << divergence_count << "\033[0m\n";
                }

            }

            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }

        return;
    }

    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        return;
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveKeyFramesAndFactor()
    {
        Eigen::Affine3f pres_tf_scan_to_map = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        if (m_awv_mos.KeyframeSelection(pres_tf_scan_to_map, timeLaserInfoCur) == false)
            return false;

        // if(cloudInfo.odomAvailable == false)
        //     return false;

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;

        thisPose3D.x = transformTobeMapped[3];
        thisPose3D.y = transformTobeMapped[4];
        thisPose3D.z = transformTobeMapped[5];
        m_i_current_keyframe_index = cloudKeyPoses3D->size(); // this can be used as index
        thisPose3D.intensity = m_i_current_keyframe_index;
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = m_i_current_keyframe_index; // this can be used as index
        thisPose6D.roll  = transformTobeMapped[0];
        thisPose6D.pitch = transformTobeMapped[1];
        thisPose6D.yaw   = transformTobeMapped[2];
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // save path for visualization
        updatePath(thisPose6D);

        return true;
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);

        return;
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);       
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        nav_msgs::Odometry initial_pose_msg;
        initial_pose_msg.header.stamp = timeLaserInfoStamp;
        initial_pose_msg.header.frame_id = odometryFrame;
        initial_pose_msg.child_frame_id = "init_odom_mapping";
        initial_pose_msg.pose.pose.position.x = InitialPose[3];
        initial_pose_msg.pose.pose.position.y = InitialPose[4];
        initial_pose_msg.pose.pose.position.z = InitialPose[5];
        initial_pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(InitialPose[0], InitialPose[1], InitialPose[2]);
        pubLaserOdometryInitial.publish(initial_pose_msg);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            transformincremental[0] = roll;
            transformincremental[1] = pitch;
            transformincremental[2] = yaw;
            transformincremental[3] = x;
            transformincremental[4] = y;
            transformincremental[5] = z;
            m_tf_scan_increase_to_map = increOdomAffine;
        } 
        else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            transformincremental[0] = roll;
            transformincremental[1] = pitch;
            transformincremental[2] = yaw;
            transformincremental[3] = x;
            transformincremental[4] = y;
            transformincremental[5] = z;
            m_tf_scan_increase_to_map = increOdomAffine;
            if (cloudInfo.imuAvailable == true)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);

        return;
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        // publish key poses
        if (pubKeyPoses.getNumSubscribers() != 0)
        {
            publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        }

        // publish surround key poses
        if (m_pubSrroundKeyPoses.getNumSubscribers() != 0)
        {
            publishCloud(m_pubSrroundKeyPoses, m_surroundingKeyPoses, timeLaserInfoStamp, odometryFrame);
        }

        // Publish surrounding key frames
        if (pubRecentKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointTypeMOS>::Ptr surrounding_keyframes_combine(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            *surrounding_keyframes_combine += *laserCloudSurfFromMapDS;
            *surrounding_keyframes_combine += *laserCloudCornerFromMapDS;
            publishCloud(pubRecentKeyFrames, surrounding_keyframes_combine, timeLaserInfoStamp, odometryFrame);
        }
        
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointTypeMOS>::Ptr cloudOut(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            for(auto& point: laserCloudCornerLastDS->points)
            {
                point.intensity = 1;
            }
            for(auto& point: laserCloudSurfLastDS->points)
            {
                point.intensity = 0;
            }
            *cloudOut += *transformPointCloudMOS(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloudMOS(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
            
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }

        // publish segmented scan
        if (m_pub_segmented_query_scan_all.getNumSubscribers() != 0)
        {
            publishCloud(m_pub_segmented_query_scan_all, m_pres_segmented_scan, timeLaserInfoStamp, "lidar_link");
        }
        if (m_pub_segmented_query_scan_static.getNumSubscribers() != 0 || m_pub_segmented_query_scan_dynamic.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointTypeMOS>::Ptr segmented_query_scan_static(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            pcl::PointCloud<PointTypeMOS>::Ptr segmented_query_scan_dynamic(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            for (const auto& point : m_pres_segmented_scan->points) 
            {
                if(point.r >= point.g || point.b >= point.g) // Filtering non-dynamic points
                    segmented_query_scan_static->points.push_back(point);
                else
                    segmented_query_scan_dynamic->points.push_back(point);
            }
            publishCloud(m_pub_segmented_query_scan_static, segmented_query_scan_static, timeLaserInfoStamp, "lidar_link");
            publishCloud(m_pub_segmented_query_scan_dynamic, segmented_query_scan_dynamic, timeLaserInfoStamp, "lidar_link");
        }

        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo.getNumSubscribers() != 0)
        {
            if (lastSLAMInfoPubSize != (int)cloudKeyPoses6D->size())
            {
                awv_mos_lio::cloud_info slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;
                pcl::PointCloud<PointTypeMOS>::Ptr cloudOut(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                *cloudOut += *laserCloudCornerLastDS;
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
                slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
                pcl::PointCloud<PointTypeMOS>::Ptr localMapOut(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
                *localMapOut += *laserCloudCornerFromMapDS;
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }

        // Publish TF
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));

        m_br.sendTransform(tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link"));

        static bool flag_init_tf = false;
        if(flag_init_tf == false)
        {          
            flag_init_tf = true;

            // static tf
            tf::Transform zero_static = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
            m_br.sendTransform(tf::StampedTransform(zero_static, timeLaserInfoStamp, mapFrame, odometryFrame));
        }

        return;
    }

    inline float pointDistance(PointType p)
    {
        return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    }


    inline float pointDistance(PointType p1, PointType p2)
    {
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
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

    void PriorMovingObjectSegmentation()
    {
        if(m_cfg_b_use_prior_mos == true && cloudInfo.odomAvailable == true)
        {
            bool is_keyframe = false;
            bool is_prior = true;
            m_awv_mos.RunOnlineMOS(m_pres_scanframe, m_tf_scan_initial_to_map, m_i_frame_count - 1, timeLaserInfoCur, is_keyframe, is_prior);
            m_awv_mos.GetSegmentedScan(m_pres_initial_segmented_scan);

            laserCloudCornerLast->clear();
            laserCloudSurfLast->clear();
            for (const auto& point : m_pres_initial_segmented_scan->points) 
            {
                if(point.r >= point.g || point.b >= point.g) // Filtering non-dynamic points
                {
                    if (point.intensity == 1) 
                        laserCloudCornerLast->points.push_back(point);
                    else
                        laserCloudSurfLast->points.push_back(point);
                }
            }
        }
        else
        {
            laserCloudCornerLast->clear();
            laserCloudSurfLast->clear();
            for (const auto& point : m_pres_scanframe->points) 
            {
                if (point.intensity == 1) 
                    laserCloudCornerLast->points.push_back(point);
                else
                    laserCloudSurfLast->points.push_back(point);
            }
        }

        return;
    }

    void MovingObjectSegmentation(const bool is_keyframe)
    {
        bool is_prior = false;
        if(cloudInfo.odomAvailable == true)
        {
            m_awv_mos.RunOnlineMOS(m_pres_scanframe, m_tf_scan_increase_to_map, m_i_frame_count - 1, timeLaserInfoCur, is_keyframe, is_prior);
            m_awv_mos.GetSegmentedScan(m_pres_segmented_scan);
        }
        else
        {
            m_pres_segmented_scan = m_pres_scanframe;
            for (auto& point : m_pres_segmented_scan->points) 
            {
                point.r = 0;
                point.g = 0;
                point.b = 255;
                point.a = 255;
            }
        }

        if(is_keyframe == true)
        {
            pcl::PointCloud<PointTypeMOS>::Ptr last_keyframe_corner_points(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            pcl::PointCloud<PointTypeMOS>::Ptr last_keyframe_surface_points(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            for (const auto& point : m_pres_segmented_scan->points) 
            {
                if(point.r >= point.g || point.b >= point.g) // Filtering non-dynamic points
                {
                    if (point.intensity == 1)
                        last_keyframe_corner_points->points.push_back(point);
                    else
                        last_keyframe_surface_points->points.push_back(point);
                }
            }
            cornerCloudKeyFrames.push_back(last_keyframe_corner_points);
            surfCloudKeyFrames.push_back(last_keyframe_surface_points);
        }

        return;
    }

    void run()
    {
        static tbb::global_control global_limit(tbb::global_control::max_allowed_parallelism, m_cfg_n_num_cpu_cores);

        // Runtime check variables
        static std::chrono::duration<double, std::milli> duration;
        static std::chrono::high_resolution_clock clock;
        static std::chrono::time_point<std::chrono::high_resolution_clock> total_start_time;
        static std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
        static int tmp_count = 0;
        tmp_count++;
        static double sum_total_time_ms = 0;
        double total_time_ms = 0;
        static double sum_extract_keyframe_time_ms = 0;
        double extract_keyframe_time_ms = 0;
        static double sum_prior_mos_time_ms = 0;
        double prior_mos_time_ms = 0;
        static double sum_down_time_ms = 0;
        double down_time_ms = 0;
        static double sum_registration_time_ms = 0;
        double registration_time_ms = 0;
        static double sum_save_keyframe_time_ms = 0;
        double save_keyframe_time_ms = 0;
        static double sum_correct_poses_time_ms = 0;
        double correct_poses_time_ms = 0;
        static double sum_segmentation_time_ms = 0;
        double segmentation_time_ms = 0;
        static double sum_publish_time_ms = 0;
        double publish_time_ms = 0;


        total_start_time = clock.now();

        if(m_vec_cloud_info_queue.size() == 0)
            return;

        m_mutexInputPointCloud.lock();
        cloudInfo = m_vec_cloud_info_queue.back();
        m_vec_cloud_info_queue.pop_back();
        m_mutexInputPointCloud.unlock();
        pcl::fromROSMsg(cloudInfo.cloud_feature, *m_pres_scanframe);
        timeLaserInfoStamp = cloudInfo.header.stamp;
        timeLaserInfoCur = cloudInfo.header.stamp.toSec();
        m_i_frame_count++;

        std::lock_guard<std::mutex> lock(mtx);


        updateInitialGuess();
        for (int i = 0; i < 6; ++i){
            InitialPose[i] = transformTobeMapped[i];
        }

        start_time = clock.now();
        extractSurroundingKeyFrames();
        duration = std::chrono::high_resolution_clock::now() - start_time;
        extract_keyframe_time_ms = duration.count();
        sum_extract_keyframe_time_ms += extract_keyframe_time_ms;

        start_time = clock.now();
        PriorMovingObjectSegmentation();
        duration = std::chrono::high_resolution_clock::now() - start_time;
        prior_mos_time_ms = duration.count();
        sum_prior_mos_time_ms += prior_mos_time_ms;

        start_time = clock.now();
        downsampleCurrentScan();
        duration = std::chrono::high_resolution_clock::now() - start_time;       
        down_time_ms = duration.count();
        sum_down_time_ms += down_time_ms;

        start_time = clock.now();
        scan2MapOptimization();
        duration = std::chrono::high_resolution_clock::now() - start_time;
        registration_time_ms = duration.count();
        sum_registration_time_ms += registration_time_ms;

        start_time = clock.now();
        bool is_keyframe = saveKeyFramesAndFactor();
        duration = std::chrono::high_resolution_clock::now() - start_time;
        save_keyframe_time_ms = duration.count();
        sum_save_keyframe_time_ms += save_keyframe_time_ms;

        start_time = clock.now();
        publishOdometry();
        duration = std::chrono::high_resolution_clock::now() - start_time;
        correct_poses_time_ms = duration.count();
        sum_correct_poses_time_ms += correct_poses_time_ms;

        start_time = clock.now();
        MovingObjectSegmentation(is_keyframe);
        duration = std::chrono::high_resolution_clock::now() - start_time;
        segmentation_time_ms = duration.count();
        sum_segmentation_time_ms += segmentation_time_ms;

        start_time = clock.now();
        if(m_cfg_b_publish_pc == true)
        {
            publishFrames();
        }
        duration = std::chrono::high_resolution_clock::now() - start_time;
        publish_time_ms = duration.count();
        sum_publish_time_ms += publish_time_ms;


        duration = std::chrono::high_resolution_clock::now() - total_start_time;
        total_time_ms = duration.count();
        sum_total_time_ms += total_time_ms;

        int frame_id = m_i_frame_count - 1;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << frame_id << " [MOS Module Ave Time] Total: " << (sum_prior_mos_time_ms + sum_segmentation_time_ms) / m_i_frame_count << ", pri: " <<  sum_prior_mos_time_ms / m_i_frame_count << ", post: " << sum_segmentation_time_ms / m_i_frame_count << "\n"; 
        std::cout << frame_id << " [SM Module Ave Time] Total: " << (sum_total_time_ms - sum_prior_mos_time_ms - sum_segmentation_time_ms) / m_i_frame_count << "\n"; 

        return;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    while (ros::ok())
    {
        // 콜백 처리
        ros::spinOnce();
        MO.run();
        ros::Duration(0.001).sleep();
    }

    return 0;
}
