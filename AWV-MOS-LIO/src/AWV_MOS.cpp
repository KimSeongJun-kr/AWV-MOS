#include "AWV_MOS.h"

AWV_MOS::AWV_MOS()
{
        // AWV_MOS config Params       
        // - Topic namespace
        nh.param<std::string>("awv_mos/m_cfg_s_output_pc_namespace", m_cfg_s_output_pc_namespace, "/awv_mos/mos_pc");
        // - LiDAR characteristic params
        nh.param<float>("awv_mos/m_cfg_f_lidar_horizontal_resolution_deg", m_cfg_f_lidar_horizontal_resolution_deg, -1);
        nh.param<float>("awv_mos/m_cfg_f_lidar_vertical_resolution_deg", m_cfg_f_lidar_vertical_resolution_deg, -1);
        nh.param<float>("awv_mos/m_cfg_f_lidar_vertical_fov_upper_bound_deg", m_cfg_f_lidar_vertical_fov_upper_bound_deg, -1);
        nh.param<float>("awv_mos/m_cfg_f_lidar_vertical_fov_lower_bound_deg", m_cfg_f_lidar_vertical_fov_lower_bound_deg, -1);
        // - Prior MOS update option
        nh.param<bool>("awv_mos/m_cfg_b_use_prior_mos", m_cfg_b_use_prior_mos, false);
        nh.param<float>("awv_mos/m_cfg_f_imu_odom_trans_err_std_m", m_cfg_f_imu_odom_trans_err_std_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_imu_odom_rot_err_std_rad", m_cfg_f_imu_odom_rot_err_std_rad, -1);
        // - Reference frame params
        nh.param<float>("awv_mos/m_cfg_f_keyframe_translation_threshold_m", m_cfg_f_keyframe_translation_threshold_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_keyframe_rotation_threshold_rad", m_cfg_f_keyframe_rotation_threshold_rad, -1);
        nh.param<float>("awv_mos/m_cfg_f_keyframe_time_threshold_s", m_cfg_f_keyframe_time_threshold_s, -1);
        nh.param<bool>("awv_mos/m_cfg_b_use_ref_frame_instant_charge", m_cfg_b_use_ref_frame_instant_charge, false);
        nh.param<int>("awv_mos/m_cfg_n_mos_ref_frame_size", m_cfg_n_mos_ref_frame_size, -1);
        // - Point-to-window comparision params
        nh.param<float>("awv_mos/m_cfg_f_meas_range_std_m", m_cfg_f_meas_range_std_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_meas_theta_std_rad", m_cfg_f_meas_theta_std_rad, -1);
        nh.param<float>("awv_mos/m_cfg_f_meas_phi_std_rad", m_cfg_f_meas_phi_std_rad, -1);
        nh.param<float>("awv_mos/m_cfg_f_scan_matching_trans_err_std_m", m_cfg_f_scan_matching_trans_err_std_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_scan_matching_rot_err_std_rad", m_cfg_f_scan_matching_rot_err_std_rad, -1);
        nh.param<float>("awv_mos/m_cfg_f_range_image_observation_window_u_angle_deg_min", m_cfg_f_range_image_observation_window_u_angle_deg_min, -1);
        nh.param<float>("awv_mos/m_cfg_f_range_image_observation_window_u_angle_deg_max", m_cfg_f_range_image_observation_window_u_angle_deg_max, -1);
        nh.param<float>("awv_mos/m_cfg_f_range_image_observation_window_v_angle_deg_min", m_cfg_f_range_image_observation_window_v_angle_deg_min, -1);
        nh.param<float>("awv_mos/m_cfg_f_range_image_observation_window_v_angle_deg_max", m_cfg_f_range_image_observation_window_v_angle_deg_max, -1);
        nh.param<float>("awv_mos/m_cfg_f_range_image_z_correction", m_cfg_f_range_image_z_correction, -1);
        nh.param<float>("awv_mos/m_cfg_f_range_image_min_dist_m", m_cfg_f_range_image_min_dist_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_range_image_min_height_m", m_cfg_f_range_image_min_height_m, -1);
        nh.param<bool>("awv_mos/m_cfg_b_use_range_image_noise_filtering", m_cfg_b_use_range_image_noise_filtering, false);
        nh.param<float>("awv_mos/m_cfg_f_range_image_noise_filtering_min_diff_m", m_cfg_f_range_image_noise_filtering_min_diff_m, -1);
        // - Motion belief calculation params
        nh.param<float>("awv_mos/m_cfg_f_moving_confidence", m_cfg_f_moving_confidence, -1);
        nh.param<float>("awv_mos/m_cfg_f_static_confidence", m_cfg_f_static_confidence, -1);
        // - Object Scale Test params
        nh.param<bool>("awv_mos/m_cfg_b_use_object_scale_test", m_cfg_b_use_object_scale_test, false);
        nh.param<float>("awv_mos/m_cfg_f_object_scale_test_valid_visible_range_m", m_cfg_f_object_scale_test_valid_visible_range_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_object_scale_test_min_height_m", m_cfg_f_object_scale_test_min_height_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_object_scale_test_min_visible_area_m2", m_cfg_f_object_scale_test_min_visible_area_m2, -1);
        nh.param<float>("awv_mos/m_cfg_f_object_scale_test_point_search_radius_m", m_cfg_f_object_scale_test_point_search_radius_m, -1);
        // - Region Growing params
        nh.param<bool>("awv_mos/m_cfg_b_use_region_growing", m_cfg_b_use_region_growing, false);
        nh.param<float>("awv_mos/m_cfg_f_region_growing_voxel_leaf_size_m", m_cfg_f_region_growing_voxel_leaf_size_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_region_growing_max_iteration", m_cfg_f_region_growing_max_iteration, -1);
        nh.param<float>("awv_mos/m_cfg_f_region_growing_point_search_radius_m", m_cfg_f_region_growing_point_search_radius_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_region_growing_ground_filter_height_m", m_cfg_f_region_growing_ground_filter_height_m, -1);
        // - Scan maching weight param
        nh.param<float>("awv_mos/m_cfg_f_static_weight_ratio", m_cfg_f_static_weight_ratio, -1);
        // - ROS msg publish params 
        nh.param<bool>("awv_mos/m_cfg_b_publish_pc", m_cfg_b_publish_pc, false);
        // - CPU Params
        nh.param<int>("awv_mos/m_cfg_n_num_cpu_cores", m_cfg_n_num_cpu_cores, -1);
        // - Prediction write
        nh.param<bool>("awv_mos/m_cfg_b_use_prediction_write", m_cfg_b_use_prediction_write, false);
        // - Mapping
        nh.param<int>("awv_mos/m_cfg_i_mapping_start_frame_limit", m_cfg_i_mapping_start_frame_limit, -1);
        nh.param<int>("awv_mos/m_cfg_i_mapping_end_frame_limit", m_cfg_i_mapping_end_frame_limit, -1);
        nh.param<bool>("awv_mos/m_cfg_b_mapping_use_save_map", m_cfg_b_mapping_use_save_map, false);
        nh.param<bool>("awv_mos/m_cfg_b_mapping_use_only_keyframes", m_cfg_b_mapping_use_only_keyframes, false);
        nh.param<bool>("awv_mos/m_cfg_b_mapping_use_mos_backward_update", m_cfg_b_mapping_use_mos_backward_update, false);
        nh.param<float>("awv_mos/m_cfg_f_mapping_section_division_length_m", m_cfg_f_mapping_section_division_length_m, -1);
        nh.param<float>("awv_mos/m_cfg_f_mapping_voxel_leaf_size_m", m_cfg_f_mapping_voxel_leaf_size_m, -1);
        nh.param<bool>("awv_mos/m_cfg_b_mapping_use_visualization", m_cfg_b_mapping_use_visualization, false);

        if (!nh.getParam("m_config_file_path", m_config_file_path)){ROS_WARN("Fail to get param - m_config_file_path");}
        if (!nh.getParam("m_log_write_path", m_log_write_path)){ROS_WARN("Fail to get param - m_log_write_path");}


    float vertical_fov = m_cfg_f_lidar_vertical_fov_upper_bound_deg - m_cfg_f_lidar_vertical_fov_lower_bound_deg;
	m_num_range_image_cols = (int)(360. / m_cfg_f_lidar_horizontal_resolution_deg) + 1;
	m_num_range_image_rows = (int)(vertical_fov / m_cfg_f_lidar_vertical_resolution_deg) + 1;
	m_num_range_image_pixels = m_num_range_image_cols * m_num_range_image_rows;

    m_kdtree_scan_moving = boost::make_shared<pcl::KdTreeFLANN<PointTypeMOS>>();
	m_kdtree_scan_unknown = boost::make_shared<pcl::KdTreeFLANN<PointTypeMOS>>();
	m_voxel_grid_filter_region_growing.setLeafSize(m_cfg_f_region_growing_voxel_leaf_size_m, 
													m_cfg_f_region_growing_voxel_leaf_size_m, 
													m_cfg_f_region_growing_voxel_leaf_size_m);
    m_deq_frames_container = std::make_shared<std::deque<ScanFrame>>();
}

AWV_MOS::~AWV_MOS(){}


void AWV_MOS::RunOnlineMOS(const pcl::PointCloud<PointTypeMOS>::Ptr& i_scan, 
                        const Eigen::Affine3f& i_tf_frame_to_map, 
                        const int& i_frame_id,
                        const double& i_time_s,
                        const bool& i_is_keyframe,
                        const bool& i_is_prior)
{
    m_query_frame = SegmentMovingObject(i_scan, i_tf_frame_to_map, i_frame_id, i_time_s, i_is_keyframe, i_is_prior);

    if(m_cfg_b_use_object_scale_test)
        ObjectScaleTest(m_query_frame);

    if(m_cfg_b_use_region_growing)
        RegionGrowing(m_query_frame);

    ManageBuffer(m_query_frame, i_is_keyframe);

    return;
}

void AWV_MOS::RunStaticMapping(const std::vector<pcl::PointCloud<PointTypeMOS>::Ptr>& i_scans, const std::vector<Eigen::Affine3f>& i_poses, const std::vector<int>& i_frames_id, const std::vector<double>& i_times, pcl::PointCloud<PointTypeMOS>::Ptr& o_static_map, pcl::PointCloud<PointTypeMOS>::Ptr& o_dynamic_map)
{
    if(i_scans.size() != i_times.size() || i_poses.size() != i_times.size() || i_frames_id.size() != i_times.size())
        std::cout << "scans, poses, times size are not mached. scans size: " << i_scans.size() << ", poses size: " << i_poses.size() << ", times size: " << i_times.size() << ", frames id size: " << i_frames_id.size() << "\n";

    std::deque<ScanFrame> ref_frames;
    std::chrono::duration<double, std::milli> duration;
    std::chrono::high_resolution_clock clock;
	int count;

    // Regist frames
    double sum_regist_time_ms = 0;
    double regist_time_ms = 0;
    count = 0;
    bool is_keyframe = true;
    ScanFrame first_scan_frame = InitScanFrame(i_scans[0], i_poses[0], i_frames_id[0], i_times[0], is_keyframe);
    m_deq_frames_container->push_back(first_scan_frame);
    m_vec_keyframe_frame_id_list.push_back(i_frames_id[0]);
    m_vec_keyframe_frame_index_list.push_back(0);
    for(int idx = 1; idx < i_poses.size(); idx++)
    {
        count++;
        auto regist_start_time = clock.now();

        int prev_keyframe_idx = m_vec_keyframe_frame_index_list.back();
        Eigen::Affine3f prev_pose = (*m_deq_frames_container)[prev_keyframe_idx].m_tf_frame_to_map;
        double prev_time = (*m_deq_frames_container)[prev_keyframe_idx].m_time_s;
        is_keyframe = KeyframeSelection(prev_pose, prev_time, i_poses[idx], i_times[idx]);

        ScanFrame scan_frame = InitScanFrame(i_scans[idx], i_poses[idx], i_frames_id[idx], i_times[idx], is_keyframe);
        m_deq_frames_container->push_back(scan_frame);

        if(is_keyframe == true)
        {
            m_vec_keyframe_frame_id_list.push_back(i_frames_id[idx]);
            m_vec_keyframe_frame_index_list.push_back(idx);
        }

        duration = std::chrono::high_resolution_clock::now() - regist_start_time;
        regist_time_ms = duration.count();
        sum_regist_time_ms += regist_time_ms;

        float progress = static_cast<float>(idx + 1) / i_poses.size() * 100.0f;
        std::cout << "\rRegist frames Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                << progress << "% (" << (idx + 1) << "/" << i_poses.size() << ") ave time : " << sum_regist_time_ms / count << " ms" << std::flush;
    }
    std::cout << "\n";

    // Run MOS   
    double sum_mos_time_ms = 0;
    double mos_time_ms = 0;
    count = 0;
    for(int idx = 0; idx < m_deq_frames_container->size() && ros::ok(); idx++)
    {
        if(m_cfg_b_mapping_use_only_keyframes == true && (*m_deq_frames_container)[idx].m_keyframe_id == -1)
            continue;

        count++;
        auto mos_start_time = clock.now();

        ScanFrame qry_frame = (*m_deq_frames_container)[idx];
        SelectReferenceFrames(qry_frame, ref_frames);

        SegmentMovingObject(qry_frame, ref_frames);

        if(m_cfg_b_use_object_scale_test)
            ObjectScaleTest(qry_frame);

        if(m_cfg_b_use_region_growing)
            RegionGrowing(qry_frame);

        duration = std::chrono::high_resolution_clock::now() - mos_start_time;
        mos_time_ms = duration.count();
        sum_mos_time_ms += mos_time_ms;

        float progress = static_cast<float>(idx + 1) / i_poses.size() * 100.0f;
        std::cout << "\rMOS Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                << progress << "% (" << (idx + 1) << "/" << i_poses.size() << ") ave time : " << sum_mos_time_ms / count << " ms" << std::flush;
    }
    std::cout << "\n";

    if(m_cfg_b_mapping_use_save_map == true)
    {
        double sum_align_time_ms = 0;
        double align_time_ms = 0;
        count = 0;
        pcl::VoxelGrid<PointTypeMOS> voxel_filter;
        voxel_filter.setLeafSize(m_cfg_f_mapping_voxel_leaf_size_m, m_cfg_f_mapping_voxel_leaf_size_m, m_cfg_f_mapping_voxel_leaf_size_m);
        pcl::PointCloud<PointTypeMOS>::Ptr total_map(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
        total_map->points.reserve(1e9);
        for(int idx = 1; idx < i_poses.size() && ros::ok(); idx++)
        {
            if(m_cfg_b_mapping_use_only_keyframes == true && (*m_deq_frames_container)[idx].m_keyframe_id == -1)
                continue;

            count++;
            auto align_start_time = clock.now();

            ScanFrame qry_frame = (*m_deq_frames_container)[idx];

            pcl::PointCloud<PointTypeMOS>::Ptr qry_scan_ds(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            if(isEvaluationPointType == true)
            {
                pcl::PointCloud<PointTypeMOSEval>::Ptr qry_scan_ds_eval(boost::make_shared<pcl::PointCloud<PointTypeMOSEval>>());
                pcl::copyPointCloud(*qry_frame.m_scan, *qry_scan_ds_eval);
                VoxelDownSamplingPreservingLabels(qry_scan_ds_eval, m_cfg_f_mapping_voxel_leaf_size_m, *qry_scan_ds_eval);                
                pcl::copyPointCloud(*qry_scan_ds_eval, *qry_scan_ds);
            }
            else
            {
                voxel_filter.setInputCloud(qry_frame.m_scan);
                voxel_filter.filter(*qry_scan_ds);
            }

            pcl::PointCloud<PointTypeMOS>::Ptr qry_scan_aligned(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            for (const auto& point : qry_scan_ds->points) 
            {
                float range = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
                if(range < m_cfg_f_range_image_min_dist_m)
                    continue;

                qry_scan_aligned->points.push_back(point);
            }
            
            pcl::transformPointCloud(*qry_scan_aligned, *qry_scan_aligned, i_poses[idx]);
            *total_map += *qry_scan_aligned;

            if(m_cfg_b_use_prediction_write == false && m_cfg_b_mapping_use_visualization == false)
            {
                qry_frame.m_scan.reset();
                qry_frame.m_scan_ptrs.reset();
            }

            duration = std::chrono::high_resolution_clock::now() - align_start_time;
            align_time_ms = duration.count();
            sum_align_time_ms += align_time_ms;

            float progress = static_cast<float>(idx + 1) / i_poses.size() * 100.0f;
            std::cout << "\rScan align with downsampling Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                    << progress << "% (" << (idx + 1) << "/" << i_poses.size() << ") ave time : " << sum_align_time_ms / count << " ms" << std::flush;
        }
        std::cout << "\n";

        pcl::PointCloud<PointTypeMOS>::Ptr static_map(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
        pcl::PointCloud<PointTypeMOS>::Ptr dynamic_map(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
        if(isEvaluationPointType == true)
        {
            pcl::PointCloud<PointTypeMOSEval>::Ptr total_map_eval(boost::make_shared<pcl::PointCloud<PointTypeMOSEval>>());
            pcl::copyPointCloud(*total_map, *total_map_eval);
            // VoxelDownSamplingPreservingLabels(total_map_eval, m_cfg_f_mapping_voxel_leaf_size_m, *total_map_eval);
            VoxelDownSamplingPreservingLabelsLargeScale(total_map_eval, m_cfg_f_mapping_section_division_length_m, m_cfg_f_mapping_voxel_leaf_size_m, *total_map_eval);
            pcl::copyPointCloud(*total_map_eval, *total_map);

            for (const auto& point : total_map->points) 
            {
                if((point.r >= point.g || point.b >= point.g)) // Filtering non-dynamic points
                    static_map->points.push_back(point);
                else
                    dynamic_map->points.push_back(point);
            }
            o_static_map = static_map;
            o_dynamic_map = dynamic_map;

            EvaluteMap(total_map_eval);
        }
        else
        {
            VoxelDownSamplingLargeScale(total_map, m_cfg_f_mapping_section_division_length_m, m_cfg_f_mapping_voxel_leaf_size_m, *total_map);
            for (const auto& point : total_map->points) 
            {
                if((point.r >= point.g || point.b >= point.g)) // Filtering non-dynamic points
                    static_map->points.push_back(point);
                else
                    dynamic_map->points.push_back(point);
            }
            o_static_map = static_map;
            o_dynamic_map = dynamic_map;
        }
    }

    return;
}

void AWV_MOS::SaveConfigParams()
{
    size_t last_slash_idx;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S_");

    // Write config file log
    last_slash_idx = m_config_file_path.find_last_of("/\\");
    std::string config_file_name;
    if (last_slash_idx != std::string::npos)
        config_file_name = m_config_file_path.substr(last_slash_idx + 1);
    else
        config_file_name = m_config_file_path;

    std::string config_log_file_name = ss.str() + config_file_name;
    std::string config_log_file_path = m_log_write_path + config_log_file_name;

    std::ifstream config_src(m_config_file_path, std::ios::binary);
    std::ofstream config_dst(config_log_file_path, std::ios::binary);
    if (!config_src.is_open()) 
        ROS_WARN("Faile to open path: %s", m_config_file_path.c_str());
    if (!config_dst.is_open()) 
        ROS_WARN("Faile to open path: %s", config_log_file_path.c_str());
    config_dst << config_src.rdbuf();
    config_src.close();
    config_dst.close();

    return;
}

void AWV_MOS::GetSegmentedScan(pcl::PointCloud<PointTypeMOS>::Ptr& o_segmented_scan)
{
    if(m_query_frame.m_scan != nullptr)
        o_segmented_scan = m_query_frame.m_scan;
    return;
}

bool AWV_MOS::KeyframeSelection(const Eigen::Affine3f& i_tf_frame_to_map, const double& i_time_scanframe)
{
    if ((m_cfg_b_use_ref_frame_instant_charge == true &&
        m_deq_reference_frame_buffer.size() < m_cfg_n_mos_ref_frame_size) ||
        m_deq_reference_frame_buffer.size() == 0)
        return true;

    Eigen::Affine3f transStart = m_deq_reference_frame_buffer.back().m_tf_frame_to_map;
    Eigen::Affine3f transFinal = i_tf_frame_to_map;
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    double prev_time_s = m_deq_reference_frame_buffer.back().m_time_s;
    if (abs(roll)  < m_cfg_f_keyframe_rotation_threshold_rad &&
        abs(pitch) < m_cfg_f_keyframe_rotation_threshold_rad && 
        abs(yaw)   < m_cfg_f_keyframe_rotation_threshold_rad &&
        sqrt(x*x + y*y + z*z) < m_cfg_f_keyframe_translation_threshold_m &&
        i_time_scanframe - prev_time_s < m_cfg_f_keyframe_time_threshold_s)
            return false;

    return true;
}

bool AWV_MOS::KeyframeSelection(const Eigen::Affine3f& i_tf_prev_frame_to_map, const double& i_time_prev_scanframe, const Eigen::Affine3f& i_tf_pres_frame_to_map, const double& i_time_pres_scanframe)
{

    Eigen::Affine3f transStart = i_tf_prev_frame_to_map;
    Eigen::Affine3f transFinal = i_tf_pres_frame_to_map;
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
    double time_diff = i_time_pres_scanframe - i_time_prev_scanframe;

    if (abs(roll)  < m_cfg_f_keyframe_rotation_threshold_rad &&
        abs(pitch) < m_cfg_f_keyframe_rotation_threshold_rad && 
        abs(yaw)   < m_cfg_f_keyframe_rotation_threshold_rad &&
        sqrt(x*x + y*y + z*z) < m_cfg_f_keyframe_translation_threshold_m &&
        time_diff < m_cfg_f_keyframe_time_threshold_s)
            return false;

    return true;
}

void AWV_MOS::Reset()
{
    m_deq_reference_frame_buffer.clear();
    
    return;
}

void AWV_MOS::WritePrediction(const pcl::PointCloud<PointTypeMOS>::Ptr& i_segmented_scan, const std::string& i_save_path)
{
    std::ofstream out;
    out.open(i_save_path, std::ios::out | std::ios::binary);
    if (!out.is_open()) 
    {
        std::cout << "[WritePrediction] Fail to open i_save_path: " << i_save_path << "\n";
        return;
    }
    
    for(int i = 0; i < i_segmented_scan->points.size(); i++)
    {
        // Moving point check
		uint8_t static_belief = i_segmented_scan->points[i].r;
		uint8_t moving_belief = i_segmented_scan->points[i].g;
		uint8_t unknown_belief = i_segmented_scan->points[i].b;
		if (moving_belief > static_belief && moving_belief > unknown_belief) // moving filter
		{
            int tmp = 251;
            out.write((char*)&tmp, sizeof(int));   
		}
        else
        {
            int tmp = 9;
            out.write((char*)&tmp, sizeof(int));
        }
    }

    // std::cout << "[WritePrediction] Prediction write done. path: " + i_save_path + "\n";

    return;
}

ScanFrame AWV_MOS::SegmentMovingObject(const pcl::PointCloud<PointTypeMOS>::Ptr& i_scan, const Eigen::Affine3f& i_tf_frame_to_map, const int& i_frame_id, const double& i_time_s, const bool& i_is_keyframe, const bool& i_is_prior)
{
    // Initialize variables for ScanFrame setup
    int qry_scan_size = i_scan->points.size();
    std::vector<bool> init_checkker(qry_scan_size, false);
    pcl::PointCloud<PointTypeMOS>::Ptr scan(boost::make_shared<pcl::PointCloud<PointTypeMOS>>(*i_scan));
    pcl::PointCloud<PointTypeMOS*>::Ptr scan_ptrs(boost::make_shared<pcl::PointCloud<PointTypeMOS*>>());
    scan_ptrs->resize(qry_scan_size);
    Eigen::Affine3f tf_frame_to_map(i_tf_frame_to_map);
    std::shared_ptr<std::vector<float>> range_image = nullptr;
    int frame_id = i_frame_id;
    int keyframe_id = -1;
    double time_s = i_time_s;
    if(i_is_keyframe == true)
    {
        static int keyframe_count = 0;
        keyframe_id = ++keyframe_count - 1;
        range_image = std::make_shared<std::vector<float>>(m_num_range_image_pixels);
    }

	int ref_frames_num = m_deq_reference_frame_buffer.size();
	std::vector<float> vec_variance_trans(ref_frames_num, 0);
	std::vector<float> vec_variance_rot(ref_frames_num, 0);
	std::vector<Eigen::Affine3f> vec_tf_qrt_to_ref(ref_frames_num);
	std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> vec_qry_scan_aligned(ref_frames_num, nullptr);

    tbb::parallel_for
	(
        size_t(0), (size_t) ref_frames_num, [&](size_t kf_i) 
        {			
            // - compute query frame pose uncertainty w.r.t reference frame
            int ref_frame_idx = m_deq_reference_frame_buffer[kf_i].m_frame_id;
            int frame_diff = fabs(frame_id - ref_frame_idx);
            if(i_is_prior == true)
            {
                vec_variance_trans[kf_i] = frame_diff * pow(m_cfg_f_scan_matching_trans_err_std_m, 2) + pow(m_cfg_f_imu_odom_trans_err_std_m, 2);
                vec_variance_rot[kf_i] = frame_diff * pow(m_cfg_f_scan_matching_rot_err_std_rad, 2) + pow(m_cfg_f_imu_odom_rot_err_std_rad, 2);
            }
            else
            {
                vec_variance_trans[kf_i] = frame_diff * pow(m_cfg_f_scan_matching_trans_err_std_m, 2);
                vec_variance_rot[kf_i] = frame_diff * pow(m_cfg_f_scan_matching_rot_err_std_rad, 2);
            }

            // - compute frame transformation
            Eigen::Affine3f tf_qry_to_ref = m_deq_reference_frame_buffer[kf_i].m_tf_frame_to_map.inverse() * tf_frame_to_map;
            vec_tf_qrt_to_ref[kf_i] = tf_qry_to_ref;

            // - transform query frame w.r.t reference frame
            pcl::PointCloud<PointTypeMOS>::Ptr qry_scan_aligned(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            pcl::transformPointCloud(*scan, *qry_scan_aligned, tf_qry_to_ref);
            vec_qry_scan_aligned[kf_i] = qry_scan_aligned;
		}
	);

    // Initialize
    tbb::parallel_for
	(
        size_t(0), (size_t) qry_scan_size, [&](size_t po_i) 
        {
            // Set pointer cloud
            scan_ptrs->points[po_i] = &(scan->points[po_i]);

            // Initialize Belief
            scan->points[po_i].r = 0;
            scan->points[po_i].g = 0;
            scan->points[po_i].b = 255;
            scan->points[po_i].a = 255;

            if(i_is_keyframe == true)
            {
                // Calc range image pixel index
                float x = scan->points[po_i].x;
                float y = scan->points[po_i].y;
                float z = scan->points[po_i].z + m_cfg_f_range_image_z_correction;

                float r_m = sqrt(x*x + y*y + z*z);
                int u = (- atan2(y, x) * 180. / M_PI + 180.) / m_cfg_f_lidar_horizontal_resolution_deg;
                int v = (atan2(z, sqrt(x*x + y*y)) * 180. / M_PI - m_cfg_f_lidar_vertical_fov_lower_bound_deg) / m_cfg_f_lidar_vertical_resolution_deg;
                v = std::clamp(v, 0, m_num_range_image_rows - 1);
                int index = u + v * m_num_range_image_cols;

                // Set range image pixel value
                if (r_m > m_cfg_f_range_image_min_dist_m && z > m_cfg_f_range_image_min_height_m && 
                    ((*range_image)[index] <= 0.0 || (*range_image)[index] > r_m))
                {
                    (*range_image)[index] = r_m;
                }
            }
        }
    );

    // RangeImgae Noise Filtering
    if(m_cfg_b_use_range_image_noise_filtering == true && i_is_keyframe == true)
    {
        std::shared_ptr<std::vector<float>> range_image_filtered = nullptr;
        range_image_filtered = std::make_shared<std::vector<float>>(m_num_range_image_pixels);
        RangeImageNoiseFiltering(range_image, range_image_filtered);
        range_image = range_image_filtered;
    }

    tbb::parallel_for
	(
        size_t(0), (size_t) qry_scan_size * ref_frames_num, [&](size_t i) 
        {
            int kf_i = i / qry_scan_size;
            int po_i = i % qry_scan_size;

            PointTypeMOS* p = &(scan->points[po_i]); // point
            PointTypeMOS* pa = &(vec_qry_scan_aligned[kf_i]->points[po_i]); // point alinged

            // unit: meter
            float p_x = p->x;
            float p_y = p->y;
            float p_z = p->z;

            float x2 = p_x * p_x;
            float y2 = p_y * p_y;
            float z2 = p_z * p_z;

            float p_r = sqrt(x2 + y2 + z2);
            float p_r2 = p_r * p_r;

            // unit: radian
            float p_theta = acos(p_z / p_r);
            float p_phi = atan2(p_y, p_x);

            // unit: meter
            float pa_x = pa->x;
            float pa_y = pa->y;
            float pa_z = pa->z;
            float pac_z = pa_z + m_cfg_f_range_image_z_correction;

            float pa_x2 = pa_x * pa_x;
            float pa_y2 = pa_y * pa_y;
            float pa_z2 = pa_z * pa_z;
            float pac_z2 = pac_z * pac_z;

            // unit: meter
            float pa_r = sqrt(pa_x2 + pa_y2 + pa_z2);
            float pac_r = sqrt(pa_x2 + pa_y2 + pac_z2);

            // unit: radian
            // float pa_theta = acos(pa_z / pa_r);
            float pac_theta = acos(pac_z / pac_r);
            float pa_phi = atan2(pa_y, pa_x);

            float ref_vert_angle_deg = - pac_theta * RADtoDEG + 90.;
            if(ref_vert_angle_deg > m_cfg_f_lidar_vertical_fov_upper_bound_deg + m_cfg_f_lidar_vertical_resolution_deg || ref_vert_angle_deg < m_cfg_f_lidar_vertical_fov_lower_bound_deg - m_cfg_f_lidar_vertical_resolution_deg)
                return;

            // - Compute point uncertainty
            // -- Compute local frame uncertainty of query point cased by pose uncertainty
            float var_trans = vec_variance_trans[kf_i];
            float var_rot = vec_variance_rot[kf_i];

            float e00 = var_rot * (y2 + z2) + var_trans;
            float e01 = -var_rot * p_x * p_y;
            float e02 = -var_rot * p_x * p_z;
            float e11 = var_rot * (x2 + z2) + var_trans;
            float e12 = -var_rot * p_y * p_z;
            float e22 = -var_rot * (x2 + y2) + var_trans;
            Eigen::Matrix3f cov_point_uncertainty_by_pose;
            cov_point_uncertainty_by_pose << e00, e01, e02, 
                                            e01, e11, e12,
                                            e02, e12, e22;

            // -- Compute local frame uncertainty of query point cased by measurement uncertainty
            float var_r = m_cfg_f_meas_range_std_m * m_cfg_f_meas_range_std_m;
            float var_theta = m_cfg_f_meas_theta_std_rad * m_cfg_f_meas_theta_std_rad;
            float var_phi = m_cfg_f_meas_phi_std_rad * m_cfg_f_meas_phi_std_rad;

            float cos_t = cos(p_theta);
            float sin_t = sin(p_theta);
            float cos_p = cos(p_phi);
            float sin_p = sin(p_phi);
            float cos_t2 = cos_t*cos_t;
            float sin_t2 = sin_t*sin_t;
            float cos_p2 = cos_p*cos_p;
            float sin_p2 = sin_p*sin_p;
            float c1 = p_r2 * var_theta * cos_t2;
            float c2 = p_r2 * var_phi * sin_t2;
            float c3 = var_r * sin_t2;
            float c4 = (var_r - p_r2 * var_theta) * sin_t * cos_t;

            e00 = (c1 + c3) * cos_p2 + c2 * sin_p2;
            e01 = (c1 - c2 + c3) * sin_p * cos_p;
            e02 = c4 * cos_p;
            e11 = (c1 + c3) * sin_p2 + c2 * cos_p2;
            e12 = c4 * sin_p;
            e22 = var_r * cos_t2 + p_r2 * var_theta * sin_t2;
            Eigen::Matrix3f cov_point_uncertainty_by_measurement;
            cov_point_uncertainty_by_measurement << e00, e01, e02, 
                                                    e01, e11, e12,
                                                    e02, e12, e22;

            // -- Compute query point uncertainty w.r.t reference frame in cartesian coordinate system 
            Eigen::Matrix3f rot = vec_tf_qrt_to_ref[kf_i].linear();
            Eigen::Matrix3f cov_point_uncertainty_cartesian;
            cov_point_uncertainty_cartesian = rot * (cov_point_uncertainty_by_pose + cov_point_uncertainty_by_measurement) * rot.transpose();

            // -- Compute query point uncertainty w.r.t reference frame in spherical coordinate system
            float cov00 = cov_point_uncertainty_cartesian(0,0);
            float cov01 = cov_point_uncertainty_cartesian(0,1);
            float cov02 = cov_point_uncertainty_cartesian(0,2);
            float cov11 = cov_point_uncertainty_cartesian(1,1);
            float cov12 = cov_point_uncertainty_cartesian(1,2);
            float cov22 = cov_point_uncertainty_cartesian(2,2);

            c1 = cov00 * pa_x + cov01 * pa_y + cov02 * pa_z;
            c2 = cov01 * pa_x + cov11 * pa_y + cov12 * pa_z;
            c3 = cov02 * pa_x + cov12 * pa_y + cov22 * pa_z;
            c4 = pa_r*pa_r - pa_z2;
            float c5 = pa_x2 + pa_y2;
            float c6 = pa_z * (cov00 * pa_x + cov01 * pa_y);
            float c7 = pa_z * (cov01 * pa_x + cov11 * pa_y);

            e00 = (pa_x*c1 + pa_y*c2 + pa_z*c3) / (pow(pa_r,2) + FLT_MIN);
            // e01 = (pa_z*(pa_x*c1 + pa_y*c2) - c4 * c3) / (pow(pa_r,3) * sqrt(c4));
            // e02 = (pa_x*c2 - pa_y*c1) / (pa_r*c5);
            e11 = (c4 * (c4 * cov22 - 2 * cov02 * pa_x * pa_z - 2 * cov12 * pa_y * pa_z) + pa_z * (c6 * pa_x + c7 * pa_y)) / (pow(pa_r,4) * c4 + FLT_MIN);
            // e12 = (pa_x * (-cov12 * c4 + c7) - pa_y * (-cov02 * c4 + c6)) / (pow(pa_r,2) * sqrt(c4) * c5);
            e22 = (cov00 * pa_y2 - 2 * cov01 * pa_x * pa_y + cov11 * pa_x2) / (pow(c5,2) + FLT_MIN);
            // Eigen::Matrix3f cov_point_uncertainty_spherical;
            // cov_point_uncertainty_spherical << e00, e01, e02, 
            // 									e01, e11, e12,
            // 									e02, e12, e22;

            float confidence_level = 2;
            float observable_radial_distance_range_m = sqrt(fabs(e00)) * confidence_level;
            float observable_theta_range_deg = sqrt(fabs(e11)) * confidence_level * RADtoDEG;
            float observable_phi_range_deg = sqrt(fabs(e22)) * confidence_level * RADtoDEG;

            // - Perform poin-to-window comparision
            observable_theta_range_deg = std::clamp(observable_theta_range_deg, m_cfg_f_range_image_observation_window_u_angle_deg_min / 2.f, m_cfg_f_range_image_observation_window_u_angle_deg_max / 2.f);
            observable_phi_range_deg = std::clamp(observable_phi_range_deg, m_cfg_f_range_image_observation_window_v_angle_deg_min / 2.f, m_cfg_f_range_image_observation_window_v_angle_deg_max / 2.f);

            // - compute window size covered observable range on image
            float window_u_cover = observable_theta_range_deg * 2 / m_cfg_f_lidar_horizontal_resolution_deg;
            float window_v_cover = observable_phi_range_deg * 2 / m_cfg_f_lidar_vertical_resolution_deg;
            
            // - convert window size to odd number
            int window_u_cover_int = round(window_u_cover);
            int window_u_size_odd = window_u_cover_int + ((window_u_cover_int & 1) ^ 1) * (2 * (window_u_cover > window_u_cover_int) - 1);
            int window_v_cover_int = round(window_v_cover);
            int window_v_size_odd = window_v_cover_int + ((window_v_cover_int & 1) ^ 1) * (2 * (window_v_cover > window_v_cover_int) - 1);
            
            // unit: radian
            Eigen::Vector3f qry_spherical_point;
            qry_spherical_point << pac_r, pac_theta, pa_phi;

            float ref_point_r = PointToWindowComparision(m_deq_reference_frame_buffer[kf_i].m_range_image,
                                                            qry_spherical_point,
                                                            window_u_size_odd, 
                                                            window_v_size_odd,
                                                            observable_radial_distance_range_m);
            if(ref_point_r > 0.0)
            {
                // - Compute Point motion belief
                std::array<uint8_t, 3> src_belief = {p->r, p->g, p->b};

                float sigma = observable_radial_distance_range_m;
                float distance_diff_m = fabs(ref_point_r - pac_r);
                float static_prob = exp(-0.5 * pow(distance_diff_m / sigma, 2));
                float moving_prob;
                if(ref_point_r > pac_r)
                {
                	moving_prob = 1.0 - static_prob;
                }
                else
                {
                	moving_prob = 0.0;
                }

                std::array<uint8_t, 3> motion_belief =
                	{(m_cfg_f_static_confidence * static_prob * 255),
                	(m_cfg_f_moving_confidence * moving_prob * 255),
                	((1. - m_cfg_f_moving_confidence * moving_prob - m_cfg_f_static_confidence * static_prob) * 255) };

                std::array<uint8_t, 3> result_belief = DempsterCombination(src_belief, motion_belief);

                p->r = result_belief[0];
                p->g = result_belief[1];
                p->b = result_belief[2];
            }
        }
	);

    return ScanFrame(scan, scan_ptrs, tf_frame_to_map, range_image, frame_id, keyframe_id, time_s);
}

void AWV_MOS::ObjectScaleTest(ScanFrame& i_frame)
{
	// Method. KDtree based
	// - make dynamic point cloud
	pcl::PointCloud<PointTypeMOS*>::Ptr moving_ptrs(new pcl::PointCloud<PointTypeMOS*>());
	pcl::PointCloud<PointTypeMOS>::Ptr moving_points(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
	for(int pt_idx = 0; pt_idx < (int)i_frame.m_scan->points.size(); pt_idx++)
	{
		auto iter_point = &(i_frame.m_scan->points[pt_idx]);

		// Moving point check
		uint8_t static_belief = iter_point->r;
		uint8_t moving_belief = iter_point->g;
		uint8_t unknown_belief = iter_point->b;
		if (moving_belief > static_belief && moving_belief > unknown_belief) // moving filter
		{
			moving_ptrs->points.push_back(iter_point);
			moving_points->points.push_back(*iter_point);
		}
	}

	// - False dynamic rejection
	if(moving_points->points.size() > 0)
		m_kdtree_scan_moving->setInputCloud(moving_points);
	else
		return;

	std::vector<bool> verified_point_check(moving_ptrs->points.size(), false);
	std::vector<bool> rejected_point_check(moving_ptrs->points.size(), false);
	uint unverified_point_count = verified_point_check.size();
	while(unverified_point_count > 0)
	{
		for(int moving_idx = 0; moving_idx < (int)moving_ptrs->points.size(); moving_idx++)
		{
			if(verified_point_check[moving_idx] == true)
				continue;

			auto moving_point_ptr = moving_ptrs->points[moving_idx];
			std::vector<int> moving_point_search_idx;
			std::vector<float> moving_point_search_dist;

			m_kdtree_scan_moving->radiusSearch(*moving_point_ptr, m_cfg_f_object_scale_test_point_search_radius_m, moving_point_search_idx, moving_point_search_dist);

			float x = moving_point_ptr->x;
			float y = moving_point_ptr->y;
			float z = moving_point_ptr->z;

			// - Compute moving object height (min max height diff)
			float min_height_m = z;
			float max_height_m = z;
			int searched_moving_point_count = 0;
			for(int idx: moving_point_search_idx)
			{
				if(rejected_point_check[idx] == false)
				{
					float tmp_z = moving_points->points[idx].z;
					if(tmp_z < min_height_m)
						min_height_m = tmp_z;
					else if(tmp_z > max_height_m)
						max_height_m = tmp_z;
					searched_moving_point_count++;
				}
			}
			float object_height = max_height_m - min_height_m;
			
			// - Compute moving object visible area
			float radial_distance_m = sqrt(x*x + y*y + z*z) + 1e-10f;
			float theta_rad = acos(z/radial_distance_m);
            float clamped_distance_m = radial_distance_m;
            if(clamped_distance_m < m_cfg_f_object_scale_test_valid_visible_range_m)
                clamped_distance_m = m_cfg_f_object_scale_test_valid_visible_range_m;
			float point_area_m2 = clamped_distance_m * sin(theta_rad) * m_cfg_f_lidar_vertical_resolution_deg / 180.0 * M_PI 
												* radial_distance_m * m_cfg_f_lidar_horizontal_resolution_deg / 180.0 * M_PI;
			float visible_area_m2 = searched_moving_point_count * point_area_m2;

			// 2-4. Modify point motion belief
			if(visible_area_m2 < m_cfg_f_object_scale_test_min_visible_area_m2 || object_height < m_cfg_f_object_scale_test_min_height_m)
			{
				for(int point_idx: moving_point_search_idx)
				{
					if(rejected_point_check[point_idx] == false || verified_point_check[point_idx] == false)
					{
						PointTypeMOS* tmp_moving_point_ptr = moving_ptrs->points[point_idx];
						tmp_moving_point_ptr->r = 0;
						tmp_moving_point_ptr->g = 0;
						tmp_moving_point_ptr->b = 255;

						rejected_point_check[point_idx] = true;
						if(verified_point_check[point_idx] == true) // This point shoud be tested again
						{
							verified_point_check[point_idx] = false;
							unverified_point_count++;
						}
						else
						{
							verified_point_check[point_idx] = true;	
							unverified_point_count--;
						}
					}
				}
			}
			else
			{
				for(int point_idx: moving_point_search_idx)
				{
					if(verified_point_check[point_idx] == false)
					{
						if(rejected_point_check[point_idx] == true)
						{
							PointTypeMOS* tmp_moving_point_ptr = moving_ptrs->points[point_idx];
							tmp_moving_point_ptr->rgb = moving_points->points[point_idx].rgb;
							rejected_point_check[point_idx] = false;
						}
						verified_point_check[point_idx] = true;
						unverified_point_count--;
					}
				}
			}
		}
	}
	return;
}

void AWV_MOS::RegionGrowing(ScanFrame& i_frame)
{
	// - Region growing
	pcl::PointCloud<PointTypeMOS>::Ptr moving_pc(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
	pcl::PointCloud<PointTypeMOS>::Ptr static_unknown_pc(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
	pcl::PointCloud<PointTypeMOS*>::Ptr static_unknown_ptr_pc(new pcl::PointCloud<PointTypeMOS*>());
	std::vector<bool> unknown_point_check;
	for(int pt_idx = 0; pt_idx < (int)i_frame.m_scan->points.size(); pt_idx++)
	{
		auto iter_point = &(i_frame.m_scan->points[pt_idx]);

		// Moving point check
		uint8_t static_belief = iter_point->r;
		uint8_t moving_belief = iter_point->g;
		uint8_t unknown_belief = iter_point->b;

		if (moving_belief > static_belief && moving_belief > unknown_belief) // moving filter
		{
			moving_pc->points.push_back(*iter_point);
		}
		else if(static_belief > moving_belief && static_belief > unknown_belief)
		{
			static_unknown_ptr_pc->points.push_back(iter_point);
			static_unknown_pc->points.push_back(*iter_point);
			unknown_point_check.push_back(false);
		}
		else if(unknown_belief > static_belief && unknown_belief > moving_belief)
		{
			static_unknown_ptr_pc->points.push_back(iter_point);
			static_unknown_pc->points.push_back(*iter_point);
			unknown_point_check.push_back(true);
		}
	}

	m_kdtree_scan_unknown->setInputCloud(static_unknown_pc);

	std::vector<bool> extended_point_check(static_unknown_pc->points.size(), false);
	int debug_loop_count = 0;
	static int debug_max_loop = 0;
	static int debug_max_loop_idx = 0;
	for(int loop_idx = 0; loop_idx < m_cfg_f_region_growing_max_iteration; loop_idx++)
	{
		pcl::PointCloud<PointTypeMOS>::Ptr filtered_moving_pc(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
		m_voxel_grid_filter_region_growing.setInputCloud(moving_pc);
		m_voxel_grid_filter_region_growing.filter(*filtered_moving_pc);
		moving_pc->clear();

		if(filtered_moving_pc->points.size() == 0)
			break;

		for(int moving_idx = 0; moving_idx < (int)filtered_moving_pc->points.size(); moving_idx++)
		{
			PointTypeMOS moving_point = filtered_moving_pc->points[moving_idx];
			std::vector<int> unknown_point_search_idx;
			std::vector<float> unknown_point_search_dist;

			// - Compute moving object visible area
			float radial_distance_m = sqrt(moving_point.x*moving_point.x + moving_point.y*moving_point.y + moving_point.z*moving_point.z) + 1e-10f;
			float theta_rad = acos(moving_point.z/radial_distance_m);
			float point_area_m2 = radial_distance_m * sin(theta_rad) * m_cfg_f_lidar_vertical_resolution_deg / 180.0 * M_PI 
												* radial_distance_m * m_cfg_f_lidar_horizontal_resolution_deg / 180.0 * M_PI;
			float point_area_radius_expanded_m = sqrt(point_area_m2) * 2;

			float search_radius_m = m_cfg_f_region_growing_point_search_radius_m;
			if(m_cfg_f_region_growing_point_search_radius_m < point_area_radius_expanded_m)
				search_radius_m = point_area_radius_expanded_m;

			m_kdtree_scan_unknown->radiusSearch(moving_point, search_radius_m, unknown_point_search_idx, unknown_point_search_dist);

			if(unknown_point_search_idx.size() == 0)
				continue;

			int searched_point_num = 0;
			float max_height_m = moving_point.z;
			float min_height_m = moving_point.z;
			bool find_static_flag = false;
			for(int point_idx = 0; point_idx < unknown_point_search_idx.size(); point_idx++)
			{
				if(extended_point_check[unknown_point_search_idx[point_idx]] == false)
				{
					PointTypeMOS searched_unknown_point = static_unknown_pc->points[unknown_point_search_idx[point_idx]];
					if(max_height_m < searched_unknown_point.z)
						max_height_m = searched_unknown_point.z;
					if(min_height_m > searched_unknown_point.z)
						min_height_m = searched_unknown_point.z;
					searched_point_num++;
				}

				if(unknown_point_check[unknown_point_search_idx[point_idx]] == false)
				{
					find_static_flag = true;
					break;
				}
			}

			float object_height_m = fabs(max_height_m - min_height_m);

			if(find_static_flag == true ||
				(object_height_m < m_cfg_f_region_growing_ground_filter_height_m && searched_point_num >= 2) ||
				searched_point_num == 0)
				continue;

			for(int unknown_idx = 0; unknown_idx < unknown_point_search_idx.size(); unknown_idx++)
			{
				if(extended_point_check[unknown_point_search_idx[unknown_idx]] == false)
				{
					PointTypeMOS* searched_unknown_point_ptr = static_unknown_ptr_pc->points[unknown_point_search_idx[unknown_idx]];
					
					searched_unknown_point_ptr->r = 0;
					searched_unknown_point_ptr->g = ((255 + 1) / 2);
					searched_unknown_point_ptr->b = ((255 - 1) / 2);

					if(m_cfg_f_region_growing_max_iteration > 1)
						moving_pc->points.push_back(*searched_unknown_point_ptr);
					extended_point_check[unknown_point_search_idx[unknown_idx]] = true;
				}
			}
		}
	}
	return;
}

void AWV_MOS::ManageBuffer(const ScanFrame& i_frame, const bool& i_is_keyframe)
{
    if(i_is_keyframe == false)
        return;

    if(m_deq_reference_frame_buffer.size() >= m_cfg_n_mos_ref_frame_size)
        m_deq_reference_frame_buffer.pop_front();

    m_deq_reference_frame_buffer.push_back(i_frame);
    
    return;
}

ScanFrame AWV_MOS::InitScanFrame(const pcl::PointCloud<PointTypeMOS>::Ptr& i_scan, 
                                    const Eigen::Affine3f& i_tf_frame_to_map, 
                                    const int& i_frame_id, 
                                    const double& i_time_s,
                                    const bool& i_is_keyframe)
{
    int num_point = i_scan->points.size();

    // Initialize variables for ScanFrame setup
    pcl::PointCloud<PointTypeMOS>::Ptr scan = i_scan;
    pcl::PointCloud<PointTypeMOS*>::Ptr scan_ptrs(boost::make_shared<pcl::PointCloud<PointTypeMOS*>>());
    scan_ptrs->resize(num_point);
    Eigen::Affine3f tf_frame_to_map(i_tf_frame_to_map);
    std::shared_ptr<std::vector<float>> range_image = nullptr;
    int frame_id = i_frame_id;
    int keyframe_id = -1;
    double time_s = i_time_s;

    if(i_is_keyframe == true)
    {
        static int keyframe_count = 0;
        keyframe_id = ++keyframe_count - 1;
        range_image = (std::make_shared<std::vector<float>>(m_num_range_image_pixels));
    }
   
    tbb::parallel_for
	(
        size_t(0), (size_t) num_point, [&](size_t i) 
		{      
            // Initialize rgb
            scan->points[i].r = 0;
            scan->points[i].g = 0;
            scan->points[i].b = 255;
            scan->points[i].a = 255;

            // Set point pointer
            scan_ptrs->points[i] = &scan->points[i];         

            if(i_is_keyframe == true)
            {
                // Calc pixel index
                float x = scan->points[i].x;
                float y = scan->points[i].y;
                float z = scan->points[i].z + m_cfg_f_range_image_z_correction;

                float r_m = sqrt(x*x + y*y + z*z);
                int u = (- atan2(y, x) * 180. / M_PI + 180.) / m_cfg_f_lidar_horizontal_resolution_deg;
                int v = (atan2(z, sqrt(x*x + y*y)) * 180. / M_PI - m_cfg_f_lidar_vertical_fov_lower_bound_deg) / m_cfg_f_lidar_vertical_resolution_deg;
                v = std::clamp(v, 0, m_num_range_image_rows - 1);
                int index = u + v * m_num_range_image_cols;

                // Set pixel value
                if (r_m > m_cfg_f_range_image_min_dist_m && ((*range_image)[index] <= 0.0 || (*range_image)[index] > r_m))
                    (*range_image)[index] = r_m;
            }
        }
    );

    // RangeImgae Noise Filtering
    if(m_cfg_b_use_range_image_noise_filtering == true && i_is_keyframe == true)
    {
        std::shared_ptr<std::vector<float>> range_image_filtered = nullptr;
        range_image_filtered = std::make_shared<std::vector<float>>(m_num_range_image_pixels);
        RangeImageNoiseFiltering(range_image, range_image_filtered);
        range_image = range_image_filtered;
    }

	return ScanFrame(scan, scan_ptrs, tf_frame_to_map, range_image, frame_id, keyframe_id, time_s);
}

void AWV_MOS::SelectReferenceFrames(const ScanFrame& i_query_frame, std::deque<ScanFrame>& o_reference_frames)
{
    int qry_id = i_query_frame.m_frame_id;

    // Forward
    static std::deque<ScanFrame> forward_ref_frames;
    static int forward_last_keyframe_id = -1;
    if(qry_id > 0)
    {   int forward_next_keyframe_id = forward_last_keyframe_id+1;
        if(forward_next_keyframe_id < m_vec_keyframe_frame_id_list.size())
        {
            int forward_next_frame_id = m_vec_keyframe_frame_id_list[forward_next_keyframe_id];
            if(forward_next_frame_id < qry_id)
            {
                forward_last_keyframe_id = forward_next_keyframe_id;
                int forward_next_frame_idx = m_vec_keyframe_frame_index_list[forward_next_keyframe_id];
                forward_ref_frames.push_back((*m_deq_frames_container)[forward_next_frame_idx]);

                if(forward_ref_frames.size() > m_cfg_n_mos_ref_frame_size)
                    forward_ref_frames.pop_front();
            }
        }
    }

    // Backward
    static std::deque<ScanFrame> backward_ref_frames;
    if(m_cfg_b_mapping_use_mos_backward_update == true)
    {
        static int backward_init = false;
        if(backward_init == false)
        {
            backward_init = true;
            int keyframe_id = -1;
            while(backward_ref_frames.size() < m_cfg_n_mos_ref_frame_size && keyframe_id < (int)m_vec_keyframe_frame_id_list.size())
            {
                keyframe_id++;
                int frame_id = m_vec_keyframe_frame_id_list[keyframe_id];
                int frame_idx = m_vec_keyframe_frame_index_list[keyframe_id];
                if(frame_id > qry_id)
                    backward_ref_frames.push_back((*m_deq_frames_container)[frame_idx]);       
            }
        }
        else if(backward_ref_frames.size() > 0)
        {
            int backward_first_frame_id = backward_ref_frames.front().m_frame_id;
            if(qry_id >= backward_first_frame_id)
            {
                backward_ref_frames.pop_front();

                if(backward_ref_frames.size() > 0)
                {
                    int backward_last_keyframe_id = backward_ref_frames.back().m_keyframe_id;
                    if(backward_last_keyframe_id < m_vec_keyframe_frame_id_list.size() - 1)
                    {
                        int backward_next_keyframe_id = backward_last_keyframe_id+1;
                        int backward_next_frame_idx = m_vec_keyframe_frame_index_list[backward_next_keyframe_id];
                        backward_ref_frames.push_back((*m_deq_frames_container)[backward_next_frame_idx]);
                    }
                }
            }
        }
    }

    std::deque<ScanFrame> reference_frames;
    for(ScanFrame frame: forward_ref_frames)
        reference_frames.push_back(frame);
    for(ScanFrame frame: backward_ref_frames)
        reference_frames.push_back(frame);
    
    o_reference_frames = reference_frames;

    return;
}

void AWV_MOS::SegmentMovingObject(const ScanFrame& i_query_frame, const std::deque<ScanFrame>& i_reference_frames)
{
    // Initialize variables for ScanFrame setup
    int qry_scan_size = i_query_frame.m_scan->points.size();
    std::vector<bool> init_checkker(qry_scan_size, false);
    pcl::PointCloud<PointTypeMOS>::Ptr scan = i_query_frame.m_scan;
    pcl::PointCloud<PointTypeMOS*>::Ptr scan_ptrs(boost::make_shared<pcl::PointCloud<PointTypeMOS*>>());
    scan_ptrs->resize(qry_scan_size);
    Eigen::Affine3f tf_frame_to_map = i_query_frame.m_tf_frame_to_map;
    int frame_id = i_query_frame.m_frame_id;

	int ref_frames_num = i_reference_frames.size();
	std::vector<float> vec_variance_trans(ref_frames_num, 0);
	std::vector<float> vec_variance_rot(ref_frames_num, 0);
	std::vector<Eigen::Affine3f> vec_tf_qrt_to_ref(ref_frames_num);
	std::vector<pcl::PointCloud<PointTypeMOS>::Ptr> vec_qry_scan_aligned(ref_frames_num, nullptr);

	// - Precalculate some infomation (pose uncertainty, frame transformation, transformed query frame)
    tbb::parallel_for
	(
        size_t(0), (size_t) ref_frames_num, [&](size_t kf_i) 
        {			
            // - compute query frame pose uncertainty w.r.t reference frame
            int ref_frame_idx = i_reference_frames[kf_i].m_frame_id;
            int frame_diff = fabs(frame_id - ref_frame_idx);

            vec_variance_trans[kf_i] = frame_diff * pow(m_cfg_f_scan_matching_trans_err_std_m, 2);
            vec_variance_rot[kf_i] = frame_diff * pow(m_cfg_f_scan_matching_rot_err_std_rad, 2);

            // - compute frame transformation
            Eigen::Affine3f tf_qry_to_ref = i_reference_frames[kf_i].m_tf_frame_to_map.inverse() * tf_frame_to_map;
            vec_tf_qrt_to_ref[kf_i] = tf_qry_to_ref;

            // - transform query frame w.r.t reference frame
            pcl::PointCloud<PointTypeMOS>::Ptr qry_scan_aligned(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            pcl::transformPointCloud(*scan, *qry_scan_aligned, tf_qry_to_ref);
            vec_qry_scan_aligned[kf_i] = qry_scan_aligned;
		}
	);

    tbb::parallel_for
	(
        size_t(0), (size_t) qry_scan_size * ref_frames_num, [&](size_t i) 
        {
            int kf_i = i / qry_scan_size;
            int po_i = i % qry_scan_size;

            PointTypeMOS* p = &(scan->points[po_i]); // point
            PointTypeMOS* pa = &(vec_qry_scan_aligned[kf_i]->points[po_i]); // point alinged

            // unit: meter
            float p_x = p->x;
            float p_y = p->y;
            float p_z = p->z;

            float x2 = p_x * p_x;
            float y2 = p_y * p_y;
            float z2 = p_z * p_z;

            float p_r = sqrt(x2 + y2 + z2);
            float p_r2 = p_r * p_r;

            // unit: radian
            float p_theta = acos(p_z / p_r);
            float p_phi = atan2(p_y, p_x);

            // unit: meter
            float pa_x = pa->x;
            float pa_y = pa->y;
            float pa_z = pa->z;
            float pac_z = pa_z + m_cfg_f_range_image_z_correction;

            float pa_x2 = pa_x * pa_x;
            float pa_y2 = pa_y * pa_y;
            float pa_z2 = pa_z * pa_z;
            float pac_z2 = pac_z * pac_z;

            // unit: meter
            float pa_r = sqrt(pa_x2 + pa_y2 + pa_z2);
            float pac_r = sqrt(pa_x2 + pa_y2 + pac_z2);

            // unit: radian
            // float pa_theta = acos(pa_z / pa_r);
            float pac_theta = acos(pac_z / pac_r);
            float pa_phi = atan2(pa_y, pa_x);

            float ref_vert_angle_deg = - pac_theta * RADtoDEG + 90.;
            if(ref_vert_angle_deg > m_cfg_f_lidar_vertical_fov_upper_bound_deg + m_cfg_f_lidar_vertical_resolution_deg || ref_vert_angle_deg < m_cfg_f_lidar_vertical_fov_lower_bound_deg - m_cfg_f_lidar_vertical_resolution_deg)
                return;

            // - Compute point uncertainty
            // -- Compute local frame uncertainty of query point cased by pose uncertainty
            float var_trans = vec_variance_trans[kf_i];
            float var_rot = vec_variance_rot[kf_i];

            float e00 = var_rot * (y2 + z2) + var_trans;
            float e01 = -var_rot * p_x * p_y;
            float e02 = -var_rot * p_x * p_z;
            float e11 = var_rot * (x2 + z2) + var_trans;
            float e12 = -var_rot * p_y * p_z;
            float e22 = -var_rot * (x2 + y2) + var_trans;
            Eigen::Matrix3f cov_point_uncertainty_by_pose;
            cov_point_uncertainty_by_pose << e00, e01, e02, 
                                            e01, e11, e12,
                                            e02, e12, e22;

            // -- Compute local frame uncertainty of query point cased by measurement uncertainty
            float var_r = m_cfg_f_meas_range_std_m * m_cfg_f_meas_range_std_m;
            float var_theta = m_cfg_f_meas_theta_std_rad * m_cfg_f_meas_theta_std_rad;
            float var_phi = m_cfg_f_meas_phi_std_rad * m_cfg_f_meas_phi_std_rad;

            float cos_t = cos(p_theta);
            float sin_t = sin(p_theta);
            float cos_p = cos(p_phi);
            float sin_p = sin(p_phi);
            float cos_t2 = cos_t*cos_t;
            float sin_t2 = sin_t*sin_t;
            float cos_p2 = cos_p*cos_p;
            float sin_p2 = sin_p*sin_p;
            float c1 = p_r2 * var_theta * cos_t2;
            float c2 = p_r2 * var_phi * sin_t2;
            float c3 = var_r * sin_t2;
            float c4 = (var_r - p_r2 * var_theta) * sin_t * cos_t;

            e00 = (c1 + c3) * cos_p2 + c2 * sin_p2;
            e01 = (c1 - c2 + c3) * sin_p * cos_p;
            e02 = c4 * cos_p;
            e11 = (c1 + c3) * sin_p2 + c2 * cos_p2;
            e12 = c4 * sin_p;
            e22 = var_r * cos_t2 + p_r2 * var_theta * sin_t2;
            Eigen::Matrix3f cov_point_uncertainty_by_measurement;
            cov_point_uncertainty_by_measurement << e00, e01, e02, 
                                                    e01, e11, e12,
                                                    e02, e12, e22;

            // -- Compute query point uncertainty w.r.t reference frame in cartesian coordinate system 
            Eigen::Matrix3f rot = vec_tf_qrt_to_ref[kf_i].linear();
            Eigen::Matrix3f cov_point_uncertainty_cartesian;
            cov_point_uncertainty_cartesian = rot * (cov_point_uncertainty_by_pose + cov_point_uncertainty_by_measurement) * rot.transpose();

            // -- Compute query point uncertainty w.r.t reference frame in spherical coordinate system
            float cov00 = cov_point_uncertainty_cartesian(0,0);
            float cov01 = cov_point_uncertainty_cartesian(0,1);
            float cov02 = cov_point_uncertainty_cartesian(0,2);
            float cov11 = cov_point_uncertainty_cartesian(1,1);
            float cov12 = cov_point_uncertainty_cartesian(1,2);
            float cov22 = cov_point_uncertainty_cartesian(2,2);

            c1 = cov00 * pa_x + cov01 * pa_y + cov02 * pa_z;
            c2 = cov01 * pa_x + cov11 * pa_y + cov12 * pa_z;
            c3 = cov02 * pa_x + cov12 * pa_y + cov22 * pa_z;
            c4 = pa_r*pa_r - pa_z2;
            float c5 = pa_x2 + pa_y2;
            float c6 = pa_z * (cov00 * pa_x + cov01 * pa_y);
            float c7 = pa_z * (cov01 * pa_x + cov11 * pa_y);

            e00 = (pa_x*c1 + pa_y*c2 + pa_z*c3) / (pow(pa_r,2) + FLT_MIN);
            // e01 = (pa_z*(pa_x*c1 + pa_y*c2) - c4 * c3) / (pow(pa_r,3) * sqrt(c4));
            // e02 = (pa_x*c2 - pa_y*c1) / (pa_r*c5);
            e11 = (c4 * (c4 * cov22 - 2 * cov02 * pa_x * pa_z - 2 * cov12 * pa_y * pa_z) + pa_z * (c6 * pa_x + c7 * pa_y)) / (pow(pa_r,4) * c4 + FLT_MIN);
            // e12 = (pa_x * (-cov12 * c4 + c7) - pa_y * (-cov02 * c4 + c6)) / (pow(pa_r,2) * sqrt(c4) * c5);
            e22 = (cov00 * pa_y2 - 2 * cov01 * pa_x * pa_y + cov11 * pa_x2) / (pow(c5,2) + FLT_MIN);
            // Eigen::Matrix3f cov_point_uncertainty_spherical;
            // cov_point_uncertainty_spherical << e00, e01, e02, 
            // 									e01, e11, e12,
            // 									e02, e12, e22;

            float confidence_level = 2;
            float observable_radial_distance_range_m = sqrt(fabs(e00)) * confidence_level;
            float observable_theta_range_deg = sqrt(fabs(e11)) * confidence_level * RADtoDEG;
            float observable_phi_range_deg = sqrt(fabs(e22)) * confidence_level * RADtoDEG;

            // - Perform poin-to-window comparision
            observable_theta_range_deg = std::clamp(observable_theta_range_deg, m_cfg_f_range_image_observation_window_u_angle_deg_min / 2.f, m_cfg_f_range_image_observation_window_u_angle_deg_max / 2.f);
            observable_phi_range_deg = std::clamp(observable_phi_range_deg, m_cfg_f_range_image_observation_window_v_angle_deg_min / 2.f, m_cfg_f_range_image_observation_window_v_angle_deg_max / 2.f);

            // - compute window size covered observable range on image
            float window_u_cover = observable_theta_range_deg * 2 / m_cfg_f_lidar_horizontal_resolution_deg;
            float window_v_cover = observable_phi_range_deg * 2 / m_cfg_f_lidar_vertical_resolution_deg;
            
            // - convert window size to odd number
            int window_u_cover_int = round(window_u_cover);
            int window_u_size_odd = window_u_cover_int + ((window_u_cover_int & 1) ^ 1) * (2 * (window_u_cover > window_u_cover_int) - 1);
            int window_v_cover_int = round(window_v_cover);
            int window_v_size_odd = window_v_cover_int + ((window_v_cover_int & 1) ^ 1) * (2 * (window_v_cover > window_v_cover_int) - 1);

            // unit: radian
            Eigen::Vector3f qry_spherical_point;
            qry_spherical_point << pac_r, pac_theta, pa_phi;

            float ref_point_r = PointToWindowComparision(i_reference_frames[kf_i].m_range_image,
                                                            qry_spherical_point,
                                                            window_u_size_odd, 
                                                            window_v_size_odd,
                                                            observable_radial_distance_range_m);
            if(ref_point_r > 0.0)
            {
                // - Compute Point motion belief
                std::array<uint8_t, 3> src_belief = {p->r, p->g, p->b};

                float sigma = observable_radial_distance_range_m;
                float distance_diff_m = fabs(ref_point_r - pac_r);
                float static_prob = exp(-0.5 * pow(distance_diff_m / sigma, 2));
                float moving_prob;
                if(ref_point_r > pac_r)
                {
                	moving_prob = 1.0 - static_prob;
                }
                else
                {
                	moving_prob = 0.0;
                }

                std::array<uint8_t, 3> motion_belief =
                	{(m_cfg_f_static_confidence * static_prob * 255),
                	(m_cfg_f_moving_confidence * moving_prob * 255),
                	((1. - m_cfg_f_moving_confidence * moving_prob - m_cfg_f_static_confidence * static_prob) * 255) };

                std::array<uint8_t, 3> result_belief = DempsterCombination(src_belief, motion_belief);

                p->r = result_belief[0];
                p->g = result_belief[1];
                p->b = result_belief[2];
            }
        }
	);

    return;
}

void AWV_MOS::VoxelDownSamplingPreservingLabels(const pcl::PointCloud<PointTypeMOSEval>::Ptr& i_src, const float& i_voxel_leaf_size_m, pcl::PointCloud<PointTypeMOSEval>& o_dst)
{
    if(i_src->points.size() <= 0)
        return;

    pcl::PointCloud<PointTypeMOSEval>::Ptr ptr_voxelized(new pcl::PointCloud<PointTypeMOSEval>);
    pcl::PointCloud<PointTypeMOSEval>::Ptr ptr_reassigned(new pcl::PointCloud<PointTypeMOSEval>);

    // 1. Voxelization
    pcl::VoxelGrid<PointTypeMOSEval> voxel_filter;
    voxel_filter.setInputCloud(i_src);
    voxel_filter.setLeafSize(i_voxel_leaf_size_m, i_voxel_leaf_size_m, i_voxel_leaf_size_m);
    voxel_filter.filter(*ptr_voxelized);

    // 2. Find nearest point to update label
    pcl::KdTreeFLANN<PointTypeMOSEval> kdtree;
    kdtree.setInputCloud(i_src);

    ptr_reassigned->points.reserve(ptr_voxelized->points.size());

    int K = 1;

    std::vector<int>   pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (const auto &pt: ptr_voxelized->points) {
        if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) 
        {
            auto updated = pt;
            updated.label = (*i_src)[pointIdxNKNSearch[0]].label;
            ptr_reassigned->points.emplace_back(updated);
        }
    }
    o_dst = *ptr_reassigned;
    return;
}

void AWV_MOS::VoxelDownSamplingPreservingLabelsLargeScale(const pcl::PointCloud<PointTypeMOSEval>::Ptr& i_src, const float& i_section_division_length_m, const float& i_voxel_leaf_size_m, pcl::PointCloud<PointTypeMOSEval>& o_dst) 
{       
    PointTypeMOSEval minPt, maxPt;
    pcl::getMinMax3D(*i_src, minPt, maxPt);

    int x_divisions = std::round((maxPt.x - minPt.x) / i_section_division_length_m);
    int y_divisions = std::round((maxPt.y - minPt.y) / i_section_division_length_m);

    float x_step = (maxPt.x - minPt.x) / x_divisions;
    float y_step = (maxPt.y - minPt.y) / y_divisions;

    int count = 0;
    int total_sections = x_divisions * y_divisions;
    pcl::PointCloud<PointTypeMOSEval>::Ptr total_cloud_DS(boost::make_shared<pcl::PointCloud<PointTypeMOSEval>>());
    for (int i = 0; i < x_divisions; ++i) {
        for (int j = 0; j < y_divisions; ++j) {
            if(ros::ok() == false)
                return;

            double min_x = minPt.x + i * x_step;
            double max_x = minPt.x + (i + 1) * x_step;
            double min_y = minPt.y + j * y_step;
            double max_y = minPt.y + (j + 1) * y_step;

            pcl::CropBox<PointTypeMOSEval> boxFilter;

            boxFilter.setMin(Eigen::Vector4f(min_x, min_y, -300, 1.0));
            boxFilter.setMax(Eigen::Vector4f(max_x, max_y, 300, 1.0));

            pcl::PointCloud<PointTypeMOSEval>::Ptr sub_cloud(boost::make_shared<pcl::PointCloud<PointTypeMOSEval>>());
            boxFilter.setInputCloud(i_src);
            boxFilter.filter(*sub_cloud);

            pcl::PointCloud<PointTypeMOSEval>::Ptr sub_cloud_DS(boost::make_shared<pcl::PointCloud<PointTypeMOSEval>>());
            VoxelDownSamplingPreservingLabels(sub_cloud, i_voxel_leaf_size_m, *sub_cloud_DS);
            *total_cloud_DS += *sub_cloud_DS;

            count++;
            float progress = static_cast<float>(count) / total_sections * 100.0f;
            std::cout << "\rVoxel downsampling Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                    << progress << "% (" << (count) << "/" << total_sections << ")" << std::flush;
        }
    }
    std::cout << "\n";

    o_dst = *total_cloud_DS;

    return;
}

void AWV_MOS::VoxelDownSamplingLargeScale(const pcl::PointCloud<PointTypeMOS>::Ptr& i_src, const float& i_section_division_length_m, const float& i_voxel_leaf_size_m, pcl::PointCloud<PointTypeMOS>& o_dst) 
{       
    PointTypeMOS minPt, maxPt;
    pcl::getMinMax3D(*i_src, minPt, maxPt);

    int x_divisions = std::round((maxPt.x - minPt.x) / i_section_division_length_m);
    int y_divisions = std::round((maxPt.y - minPt.y) / i_section_division_length_m);

    float x_step = (maxPt.x - minPt.x) / x_divisions;
    float y_step = (maxPt.y - minPt.y) / y_divisions;

    pcl::VoxelGrid<PointTypeMOS> voxel_filter;
    voxel_filter.setLeafSize(i_voxel_leaf_size_m, i_voxel_leaf_size_m, i_voxel_leaf_size_m);

    int count = 0;
    int total_sections = x_divisions * y_divisions;
    pcl::PointCloud<PointTypeMOS>::Ptr total_cloud_DS(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
    for (int i = 0; i < x_divisions; ++i) {
        for (int j = 0; j < y_divisions; ++j) {
            pcl::PointCloud<PointTypeMOS>::Ptr sub_cloud(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());

            double min_x = minPt.x + i * x_step;
            double max_x = minPt.x + (i + 1) * x_step;
            double min_y = minPt.y + j * y_step;
            double max_y = minPt.y + (j + 1) * y_step;

            pcl::CropBox<PointTypeMOS> boxFilter;

            boxFilter.setMin(Eigen::Vector4f(min_x, min_y, -300, 1.0));
            boxFilter.setMax(Eigen::Vector4f(max_x, max_y, 300, 1.0));

            boxFilter.setInputCloud(i_src);
            boxFilter.filter(*sub_cloud);

            pcl::PointCloud<PointTypeMOS>::Ptr sub_cloud_DS(boost::make_shared<pcl::PointCloud<PointTypeMOS>>());
            voxel_filter.setInputCloud(sub_cloud);
            voxel_filter.filter(*sub_cloud_DS);
            *total_cloud_DS += *sub_cloud_DS;

            count++;
            float progress = static_cast<float>(count) / total_sections * 100.0f;
            std::cout << "\rVoxel downsampling Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                    << progress << "% (" << (count) << "/" << total_sections << ")" << std::flush;
        }
    }
    std::cout << "\n";

    o_dst = *total_cloud_DS;    

    return;
}

void AWV_MOS::EvaluteMap(pcl::PointCloud<PointTypeMOSEval>::Ptr& i_src)
{
    int scan_TP = 0;
    int scan_TN = 0;
    int scan_FP = 0;
    int scan_FN = 0;
    
    for(int idx = 0; idx < (int)i_src->points.size(); idx++)
    {
        float progress = static_cast<float>(idx + 1) / i_src->points.size() * 100.0f;
        if((int)(progress * 100) % 100 == 0 || progress == 100.0)
            std::cout << "\rMap Evaluation Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                    << progress << "% (" << (idx + 1) << "/" << i_src->points.size() << ")" << std::flush;

		// Motion belief update
		int static_belief = i_src->points[idx].r;
		int moving_belief = i_src->points[idx].g;
		int unknown_belief = i_src->points[idx].b;

        bool predict_is_dynamic = (moving_belief > static_belief && moving_belief > unknown_belief);

        int gt_label = i_src->points[idx].label;
        bool gt_is_dynamic;

        if(gt_label >= 9 && gt_label <= 99) // static
            gt_is_dynamic = false;
        else if(gt_label >= 251 && gt_label <= 259)
            gt_is_dynamic = true;
        else
            continue;

        if(gt_is_dynamic == true && predict_is_dynamic == true)
            scan_TP++;
        else if(gt_is_dynamic == false && predict_is_dynamic == false)
            scan_TN++;
        else if(gt_is_dynamic == true && predict_is_dynamic == false)
            scan_FN++;
        else if(gt_is_dynamic == false && predict_is_dynamic == true)
            scan_FP++;
    }
    std::cout << "\n";

    double scan_IoU = (double) (scan_TP + 1) / (double) (scan_TP + scan_FP + scan_FN + 1);
    double scan_accuracy = (double) (scan_TP + scan_TN + 1) / (double) (scan_TP + scan_TN + scan_FP + scan_FN + 1);
    double scan_precision = (double) (scan_TP + 1) / (double) (scan_TP + scan_FP + 1);
    double scan_recall = (double) (scan_TP + 1) / (double) (scan_TP + scan_FN + 1);
    double scan_PR = (double) (scan_TN + 1) / (double) (scan_TN + scan_FP + 1);
    double scan_RR = (double) (scan_TP + 1) / (double) (scan_TP + scan_FN + 1);
    double scan_score = (scan_PR + scan_RR) / 2.0; 

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[EvaluteMap] num points: " << i_src->points.size() 
                << ", scan_IoU: " << scan_IoU 
                << ", precision: " << scan_precision 
                << ", recall: " << scan_recall 
                << ", TP: " << scan_TP << ", TN: " << scan_TN << ", FP: " << scan_FP << ", FN: " << scan_FN << "\n";
}


std::array<uint8_t, 3> AWV_MOS::DempsterCombination(std::array<uint8_t, 3> i_src_belief, std::array<uint8_t, 3> i_other_belief)
{
    std::array<uint8_t, 3> result_belief;

	unsigned short denominator = (unsigned short)255*255 
                        - (unsigned short)i_src_belief[0] * (unsigned short)i_other_belief[1] 
                        - (unsigned short)i_src_belief[1] * (unsigned short)i_other_belief[0];

	if (denominator == 0)
	{
		result_belief[0] = 0;
		result_belief[1] = 0;
		result_belief[2] = 255;

		return result_belief;
	}

	unsigned short tmp_0 = ((unsigned short)i_src_belief[0] * (unsigned short)i_other_belief[2]
                + (unsigned short)i_src_belief[2] * (unsigned short)i_other_belief[0]
                + (unsigned short)i_src_belief[0] * (unsigned short)i_other_belief[0]) * (unsigned short)255 / denominator;

	unsigned short tmp_1 = ((unsigned short)i_src_belief[1] * (unsigned short)i_other_belief[2]
                + (unsigned short)i_src_belief[2] * (unsigned short)i_other_belief[1]
                + (unsigned short)i_src_belief[1] * (unsigned short)i_other_belief[1]) * (unsigned short)255 / denominator;

	unsigned short tmp_2 = (unsigned short)255 - tmp_1 - tmp_0;
    
    result_belief[0] = tmp_0;
    result_belief[1] = tmp_1;
    result_belief[2] = tmp_2;

	return result_belief;
}

void AWV_MOS::RangeImageNoiseFiltering(const std::shared_ptr<std::vector<float>>& i_range_image, std::shared_ptr<std::vector<float>>& o_range_image_filtered)
{
    tbb::parallel_for
	(
        size_t(0), (size_t) m_num_range_image_pixels, [&](size_t center_pixel_idx) 
        {		
            // int center_pixel_idx = pixel_u + pixel_v * m_num_range_image_cols;
            int pixel_u =  center_pixel_idx % m_num_range_image_cols;
            int pixel_v =  center_pixel_idx / m_num_range_image_cols;


            // Search distance in hash table
            float center_pixel_r_m = (*i_range_image)[center_pixel_idx];
            float min_diff_m = INFINITY;

            for(int win_pix_u = 0; win_pix_u < 3; win_pix_u++)
            {
                for(int win_pix_v = 0; win_pix_v < 3; win_pix_v++)
                {
                    int win_pix_u_idx = pixel_u + (win_pix_u - (3 - 1) / 2);
                    if(win_pix_u_idx < 0)
                    {
                        win_pix_u_idx += m_num_range_image_cols;
                    }
                    else if(win_pix_u_idx > m_num_range_image_cols - 1)
                    {
                        win_pix_u_idx -= m_num_range_image_cols;
                    }

                    // int win_pix_u_idx = std::max(std::min(iHorizontalAngle + (win_pix_u - (i_window_u_size - 1) / 2), m_num_range_image_cols - 1), 0);
                    int win_pix_v_idx = std::clamp(pixel_v + (win_pix_v - (3 - 1) / 2), 0, m_num_range_image_rows - 1);
                    // int win_pix_v_idx = std::max(std::min(iVerticalAngle + (win_pix_v - (i_window_v_size - 1) / 2), m_num_range_image_rows - 1), 0);
                    int ref_pixel_idx = win_pix_u_idx + win_pix_v_idx * m_num_range_image_cols;

                    // Insert data
                    float ref_pixel_r_m = (*i_range_image)[ref_pixel_idx];
                    if(ref_pixel_r_m <= 0.0 || ref_pixel_idx == center_pixel_idx)
                        continue;
                    
                    float diff = abs(center_pixel_r_m - ref_pixel_r_m);
                    if(diff < min_diff_m)
                    {
                        min_diff_m = diff;
                    }                    
                }
            }

            if(min_diff_m < m_cfg_f_range_image_noise_filtering_min_diff_m)
                (*o_range_image_filtered)[center_pixel_idx] = (*i_range_image)[center_pixel_idx];
        }
    );
    return;
}


float AWV_MOS::PointToWindowComparision(const std::shared_ptr<std::vector<float>>& i_range_image, 
                                        const Eigen::Vector3f& i_spherical_point_m_rad, 
                                        const int& i_winow_u_size, 
                                        const int& i_winow_v_size, 
                                        const float& i_observavle_radial_distance_range)
{
    if (i_range_image == nullptr)
		return 0.0;

	// Comptue pixel where query point is projected 
	int pixel_u = (- i_spherical_point_m_rad(2) * RADtoDEG + 180.0) / m_cfg_f_lidar_horizontal_resolution_deg;
	int pixel_v = ((M_PI - i_spherical_point_m_rad(1)) * RADtoDEG - (m_cfg_f_lidar_vertical_fov_lower_bound_deg + 90.)) / m_cfg_f_lidar_vertical_resolution_deg;
	pixel_v = std::clamp(pixel_v, 0, m_num_range_image_rows - 1);
	int pixel_idx = pixel_u + pixel_v * m_num_range_image_cols;

	// Search distance in hash table
	float closest_dist_m = INFINITY;

	for(int win_pix_u = 0; win_pix_u < i_winow_u_size; win_pix_u++)
	{
		for(int win_pix_v = 0; win_pix_v < i_winow_v_size; win_pix_v++)
		{
			int win_pix_u_idx = pixel_u + (win_pix_u - (i_winow_u_size - 1) / 2);
			if(win_pix_u_idx < 0)
			{
				win_pix_u_idx += m_num_range_image_cols;
			}
			else if(win_pix_u_idx > m_num_range_image_cols - 1)
			{
				win_pix_u_idx -= m_num_range_image_cols;
			}

			// int win_pix_u_idx = std::max(std::min(iHorizontalAngle + (win_pix_u - (i_window_u_size - 1) / 2), m_num_range_image_cols - 1), 0);
			int win_pix_v_idx = std::clamp(pixel_v + (win_pix_v - (i_winow_v_size - 1) / 2), 0, m_num_range_image_rows - 1);
			// int win_pix_v_idx = std::max(std::min(iVerticalAngle + (win_pix_v - (i_window_v_size - 1) / 2), m_num_range_image_rows - 1), 0);
			int pix_idx = win_pix_u_idx + win_pix_v_idx * m_num_range_image_cols;

			// Insert data
			float reference_point_r_m = (*i_range_image)[pix_idx];
			if(reference_point_r_m <= 0.0)
				continue;
				
			if(reference_point_r_m < closest_dist_m)
			{
				closest_dist_m = reference_point_r_m;
			}

			if(i_spherical_point_m_rad(0) - closest_dist_m > i_observavle_radial_distance_range)
			{
				return closest_dist_m;
			}
		}
	}

	if(closest_dist_m == INFINITY)
	{
		return 0.0;
	}
	
	return closest_dist_m;
}