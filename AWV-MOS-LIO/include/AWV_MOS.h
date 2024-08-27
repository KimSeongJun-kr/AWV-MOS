#pragma once
#define PCL_NO_PRECOMPILE 

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <tbb/parallel_for.h>
#include <tbb/global_control.h>

constexpr bool isEvaluationPointType = 
#ifdef USE_EVALUATION_POINT_TYPE
    true;
#else
    false;
#endif

#ifndef _POINT_TYPE_DEFINITION_
#define _POINT_TYPE_DEFINITION_
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
#ifdef USE_EVALUATION_POINT_TYPE
    // Evaluation Expression
    using PointType = PointXYZIRTRGBL;
    using PointTypeMOS = PointXYZIRTRGBL;
    using PointTypeMOSEval = PointXYZIRTRGBL;
    using PointTypeMOSDeskew = PointXYZIRTRGBL;
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
    using PointTypeMOSEval = PointXYZIRTRGBL;
    using PointTypeMOSDeskew = PointXYZIRTRGBL;
#endif
#endif


struct ScanFrame
{
    pcl::PointCloud<PointTypeMOS>::Ptr m_scan;
    pcl::PointCloud<PointTypeMOS*>::Ptr m_scan_ptrs;
    Eigen::Affine3f m_tf_frame_to_map;
    std::shared_ptr<std::vector<float>> m_range_image;
    int m_frame_id;
    int m_keyframe_id;
    double m_time_s;
    
    ScanFrame():
        m_scan(nullptr),
        m_scan_ptrs(nullptr),
        m_tf_frame_to_map(Eigen::Affine3f()),
        m_range_image(nullptr),
        m_frame_id(-1),
        m_keyframe_id(-1),
        m_time_s(-1){}
    ScanFrame(pcl::PointCloud<PointTypeMOS>::Ptr i_scan, 
            pcl::PointCloud<PointTypeMOS*>::Ptr i_scan_ptrs, 
            Eigen::Affine3f& i_tf_frame_to_map, 
            std::shared_ptr<std::vector<float>> i_range_image,
            int& i_frame_id,
            int& i_keyframe_id,
            double& i_time_s):
        m_scan(i_scan),
        m_scan_ptrs(i_scan_ptrs),
        m_tf_frame_to_map(i_tf_frame_to_map),
        m_range_image(i_range_image),
        m_frame_id(i_frame_id),
        m_keyframe_id(i_keyframe_id),
        m_time_s(i_time_s){}
    ScanFrame(const ScanFrame& other) 
    {
        m_scan = other.m_scan;
        m_scan_ptrs = other.m_scan_ptrs;
        m_tf_frame_to_map = other.m_tf_frame_to_map;
        m_range_image = other.m_range_image;
        m_frame_id = other.m_frame_id;
        m_keyframe_id = other.m_keyframe_id;
        m_time_s = other.m_time_s;
    }
    ScanFrame& operator=(const ScanFrame& other) 
    {
        if (this == &other)
            return *this;
        m_scan = other.m_scan;
        m_scan_ptrs = other.m_scan_ptrs;
        m_tf_frame_to_map = other.m_tf_frame_to_map;
        m_range_image = other.m_range_image;
        m_frame_id = other.m_frame_id;
        m_keyframe_id = other.m_keyframe_id;
        m_time_s = other.m_time_s;
        return *this;
    }
    ~ScanFrame() {}
};

class AWV_MOS
{
public:
    AWV_MOS();
    ~AWV_MOS();

public:
    void RunOnlineMOS(const pcl::PointCloud<PointTypeMOS>::Ptr& i_scan, 
                        const Eigen::Affine3f& i_tf_frame_to_map,
                        const int& i_frame_id,
                        const double& i_time_s,
                        const bool& i_is_keyframe = false,
                        const bool& i_is_prior = false);
             
    void RunStaticMapping(const std::vector<pcl::PointCloud<PointTypeMOS>::Ptr>& i_scans, 
                        const std::vector<Eigen::Affine3f>& i_poses, 
                        const std::vector<int>& i_frames_id, 
                        const std::vector<double>& i_times, 
                        pcl::PointCloud<PointTypeMOS>::Ptr& o_static_map,
                        pcl::PointCloud<PointTypeMOS>::Ptr& o_dynamic_map);
    
public:
    void SaveConfigParams();
    bool KeyframeSelection(const Eigen::Affine3f& i_tf_frame_to_map, const double& i_time_scanframe);
    bool KeyframeSelection(const Eigen::Affine3f& i_tf_prev_frame_to_map, const double& i_time_prev_scanframe, const Eigen::Affine3f& i_tf_pres_frame_to_map, const double& i_time_pres_scanframe);
    void Reset();
    void WritePrediction(const pcl::PointCloud<PointTypeMOS>::Ptr& i_segmented_scan, const std::string& i_save_path);
    void GetSegmentedScan(pcl::PointCloud<PointTypeMOS>::Ptr& o_segmented_scan);

public:
    ros::NodeHandle nh;

    // AWV_MOS config Params
    // - Topic namespace
    std::string m_cfg_s_output_pc_namespace;
    // - LiDAR characteristic params
    float m_cfg_f_lidar_horizontal_resolution_deg;
    float m_cfg_f_lidar_vertical_resolution_deg;
    float m_cfg_f_lidar_vertical_fov_upper_bound_deg;
    float m_cfg_f_lidar_vertical_fov_lower_bound_deg;
    // - Prior MOS update option
    bool m_cfg_b_use_prior_mos;
    float m_cfg_f_imu_odom_trans_err_std_m;
    float m_cfg_f_imu_odom_rot_err_std_rad;
    // - Reference frame params
    float m_cfg_f_keyframe_translation_threshold_m;
    float m_cfg_f_keyframe_rotation_threshold_rad;
    float m_cfg_f_keyframe_time_threshold_s;
    bool m_cfg_b_use_ref_frame_instant_charge;
    int m_cfg_n_mos_ref_frame_size;
    // - Point-to-window comparision params
    float m_cfg_f_meas_range_std_m;
    float m_cfg_f_meas_theta_std_rad;
    float m_cfg_f_meas_phi_std_rad;
    float m_cfg_f_scan_matching_trans_err_std_m;
    float m_cfg_f_scan_matching_rot_err_std_rad;
    float m_cfg_f_range_image_observation_window_u_angle_deg_min;
    float m_cfg_f_range_image_observation_window_u_angle_deg_max;
    float m_cfg_f_range_image_observation_window_v_angle_deg_min;
    float m_cfg_f_range_image_observation_window_v_angle_deg_max;
    float m_cfg_f_range_image_z_correction;
    float m_cfg_f_range_image_min_dist_m;
    float m_cfg_f_range_image_min_height_m;
    bool m_cfg_b_use_range_image_noise_filtering;
    float m_cfg_f_range_image_noise_filtering_min_diff_m;
    // - Motion belief calculation params
    float m_cfg_f_moving_confidence;
    float m_cfg_f_static_confidence;
    // - Object Scale Test params
    bool m_cfg_b_use_object_scale_test;
    float m_cfg_f_object_scale_test_valid_visible_range_m;
	float m_cfg_f_object_scale_test_min_height_m;
    float m_cfg_f_object_scale_test_min_visible_area_m2;
	float m_cfg_f_object_scale_test_point_search_radius_m;
    // - Resgion Growing params
    bool m_cfg_b_use_region_growing;
    float m_cfg_f_region_growing_voxel_leaf_size_m;
    float m_cfg_f_region_growing_max_iteration; 
    float m_cfg_f_region_growing_point_search_radius_m;
    float m_cfg_f_region_growing_ground_filter_height_m;
    // - Scan maching weight param
    float m_cfg_f_static_weight_ratio;
    // - ROS msg publish params 
    bool m_cfg_b_publish_pc;
    // - CPU Params
    int m_cfg_n_num_cpu_cores;
    // - Prediction write
    bool m_cfg_b_use_prediction_write;
    // - Mapping
    int m_cfg_i_mapping_start_frame_limit;
    int m_cfg_i_mapping_end_frame_limit;
    bool m_cfg_b_mapping_use_save_map;
    bool m_cfg_b_mapping_use_only_keyframes;
    bool m_cfg_b_mapping_use_mos_backward_update;
    float m_cfg_f_mapping_section_division_length_m;
    float m_cfg_f_mapping_voxel_leaf_size_m;
    bool m_cfg_b_mapping_use_visualization;
    // - Log write params
    std::string m_log_write_path;
    // config file path
    std::string m_config_file_path;
    std::string m_mos_config_file_path;

private:
	static constexpr float RADtoDEG = 180. / M_PI;
	static constexpr float DEGtoRAD = M_PI / 180.;

    std::deque<ScanFrame> m_deq_reference_frame_buffer;
    ScanFrame m_query_frame;

	pcl::KdTreeFLANN<PointTypeMOS>::Ptr m_kdtree_scan_moving;
	pcl::KdTreeFLANN<PointTypeMOS>::Ptr m_kdtree_scan_unknown;
	pcl::VoxelGrid<PointTypeMOS> m_voxel_grid_filter_region_growing;

	int m_num_range_image_cols;
	int m_num_range_image_rows;
	int m_num_range_image_pixels;

private:
    std::shared_ptr<std::deque<ScanFrame>> m_deq_frames_container;
    std::vector<int> m_vec_keyframe_frame_id_list;
    std::vector<int> m_vec_keyframe_frame_index_list;

private:
    ScanFrame SegmentMovingObject(const pcl::PointCloud<PointTypeMOS>::Ptr& i_scan, const Eigen::Affine3f& i_tf_frame_to_map, const int& i_frame_id, const double& i_time_s, const bool& i_is_keyframe, const bool& i_is_prior);
    void ObjectScaleTest(ScanFrame& i_frame);
    void RegionGrowing(ScanFrame& i_frame);
    void ManageBuffer(const ScanFrame& i_frame, const bool& i_is_keyframe);

// Static Mapping
    ScanFrame InitScanFrame(const pcl::PointCloud<PointTypeMOS>::Ptr& i_scan, const Eigen::Affine3f& i_tf_frame_to_map, const int& i_frame_id, const double& i_time_s, const bool& i_is_keyframe);
    void SelectReferenceFrames(const ScanFrame& i_query_frame, std::deque<ScanFrame>& o_reference_frames);
    void SegmentMovingObject(const ScanFrame& i_query_frame, const std::deque<ScanFrame>& i_reference_frames);
    void VoxelDownSamplingPreservingLabels(const pcl::PointCloud<PointTypeMOSEval>::Ptr& i_src, const float& i_voxel_leaf_size_m, pcl::PointCloud<PointTypeMOSEval>& o_dst);
    void VoxelDownSamplingPreservingLabelsLargeScale(const pcl::PointCloud<PointTypeMOSEval>::Ptr& i_src, const float& i_section_division_length_m, const float& i_voxel_leaf_size_m, pcl::PointCloud<PointTypeMOSEval>& o_dst);
    void VoxelDownSamplingLargeScale(const pcl::PointCloud<PointTypeMOS>::Ptr& i_src, const float& i_section_division_length_m, const float& i_voxel_leaf_size_m, pcl::PointCloud<PointTypeMOS>& o_dst);
    void EvaluteMap(pcl::PointCloud<PointTypeMOSEval>::Ptr& i_src);

private:    
    void RangeImageNoiseFiltering(const std::shared_ptr<std::vector<float>>& i_range_image, std::shared_ptr<std::vector<float>>& o_range_image_filtered);
    float PointToWindowComparision(const std::shared_ptr<std::vector<float>>& i_range_image, 
                                    const Eigen::Vector3f& i_spherical_point_m_rad, 
                                    const int& i_winow_size_u, 
                                    const int& i_winow_size_v, 
                                    const float& i_observavle_radial_distance_range);
    std::array<unsigned char, 3> DempsterCombination(const std::array<unsigned char, 3> i_src_belief, std::array<unsigned char, 3> const i_other_belief);

};