#include <filesystem>
#include <unordered_map>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

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

using PointTypeMOSEval = PointXYZIRTRGBL;

bool GetLabelsFromBag(const std::string& i_bag_file_path, const std::string& i_scan_topic, int& i_start_frame, int& i_end_frame, std::vector<std::vector<int>>& o_gt_labels, std::vector<std::vector<int>>& o_pred_labels)
{
    std::cout << "Read bag file - start. path: " << i_bag_file_path << "\n";
    rosbag::Bag bag(i_bag_file_path);
    if(bag.isOpen() == false)
    {
        std::cout << "Fail to open bag file : " << i_bag_file_path << "\n";
        return false;        
    }
      
    rosbag::View view(bag, rosbag::TopicQuery(i_scan_topic));
    int scan_msgs_size = view.size();
    int count = 0;
    std::vector<sensor_msgs::PointCloud2::ConstPtr> scan_msgs;
    for (const auto& msg : view)
    {
        if(ros::ok() == false)
            return false;

        count++;
        sensor_msgs::PointCloud2::ConstPtr instance = msg.instantiate<sensor_msgs::PointCloud2>();
        if (instance != nullptr)
        {
            scan_msgs.push_back(instance);
            float progress = static_cast<float>(count) / scan_msgs_size * 100.0f;
            std::cout << "\rRead msgs from bag Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                    << progress << "% (" << (count) << "/" << scan_msgs_size << ")" << std::flush;
        }
    }
    std::cout << "\n";

    std::vector<std::vector<int>> gt_labels;
    std::vector<std::vector<int>> pred_labels;
    gt_labels.resize(scan_msgs_size);
    pred_labels.resize(scan_msgs_size);

    for(int frame_idx = 0; frame_idx < scan_msgs_size; frame_idx++)
    {
        if(ros::ok() == false)
            return false;

        count++;
        pcl::PointCloud<PointTypeMOSEval>::Ptr segmented_scan(boost::make_shared<pcl::PointCloud<PointTypeMOSEval>>());
        pcl::fromROSMsg(*scan_msgs[frame_idx], *segmented_scan);

        int num_of_point = segmented_scan->points.size();
        std::vector<int> gt_label, pred_label;
        gt_label.resize(num_of_point);
        pred_label.resize(num_of_point);
        
        for(int po_i = 0; po_i < (int)num_of_point; po_i++)
        {
            // Get gt label
            gt_label[po_i] = segmented_scan->points[po_i].label;

            // Get pred label
            int static_belief = segmented_scan->points[po_i].r;
            int moving_belief = segmented_scan->points[po_i].g;
            int unknown_belief = segmented_scan->points[po_i].b;

            if (moving_belief > static_belief && moving_belief > unknown_belief) // dynamic filter
                pred_label[po_i] = 251;
            else
                pred_label[po_i] = 9;
        }
        gt_labels[frame_idx] = gt_label;
        pred_labels[frame_idx] = pred_label;
        
        float progress = static_cast<float>(frame_idx+1) / scan_msgs_size * 100.0f;
        std::cout << "\rRead labels from bag Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                << progress << "% (" << (frame_idx+1) << "/" << scan_msgs_size << ")" << std::flush;
    }
    std::cout << "\n";
    bag.close();

    // Limit frames
    if(i_start_frame > 0 || i_end_frame > 0)
    {
        int limit_start_frame = 0;
        if(i_start_frame > 0 && i_start_frame < gt_labels.size())
            limit_start_frame = i_start_frame;
        int limit_end_frame = gt_labels.size() - 1;
        if(i_end_frame > 0 && i_end_frame < gt_labels.size())
            limit_end_frame = i_end_frame;

        std::vector<std::vector<int>> tmp_gt_labels;
        std::vector<std::vector<int>> tmp_pred_labels;
        for(int idx = limit_start_frame; idx <= limit_end_frame; idx++)
        {
            tmp_gt_labels.push_back(gt_labels[idx]);
            tmp_pred_labels.push_back(pred_labels[idx]);
        }
        gt_labels = tmp_gt_labels;
        pred_labels = tmp_pred_labels;
        std::cout << "Limit frames. num of selected frame: " << tmp_gt_labels.size() << "\n";
    }

    o_gt_labels = gt_labels;
    o_pred_labels = pred_labels;

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

bool GetLabelsFromKitti(const std::string& i_gt_label_folder_path, const std::string& i_pred_label_folder_path, int& i_start_frame, int& i_end_frame, std::vector<std::vector<int>>& o_gt_labels, std::vector<std::vector<int>>& o_pred_labels)
{
    std::cout << "Read kitti format label file list - start.\n";
    // - Get label file list
    std::vector<std::string> gt_label_file_list;
    std::vector<std::string> pred_label_file_list;

    if (std::filesystem::exists(i_gt_label_folder_path) == false || 
        std::filesystem::is_directory(i_gt_label_folder_path) == false)
    {
        std::cout << "gt label folder does not exist or is not a directory: " << i_gt_label_folder_path << std::endl;
        return false;
    }
    if (std::filesystem::exists(i_pred_label_folder_path) == false || 
        std::filesystem::is_directory(i_pred_label_folder_path) == false)
    {
        std::cout << "pred label folder does not exist or is not a directory: " << i_pred_label_folder_path << std::endl;
        return false;
    }

    // -- find files
    for (const auto& entry : std::filesystem::directory_iterator(i_gt_label_folder_path)) 
    {
        if (entry.is_regular_file()) 
        {
            gt_label_file_list.push_back(entry.path().string());
        }
    }
    for (const auto& entry : std::filesystem::directory_iterator(i_pred_label_folder_path)) 
    {
        if (entry.is_regular_file()) 
        {
            pred_label_file_list.push_back(entry.path().string());
        }
    }

    // -- sort list
    std::sort(gt_label_file_list.begin(), gt_label_file_list.end(), 
            [](const std::string& a, const std::string& b) 
            {return extract_number(a) < extract_number(b);});
    std::sort(pred_label_file_list.begin(), pred_label_file_list.end(), 
            [](const std::string& a, const std::string& b) 
            {return extract_number(a) < extract_number(b);});

    if(gt_label_file_list.size() != pred_label_file_list.size())
    {
        std::vector<std::string> tmp_gt_label_file_list;
        std::vector<std::string> tmp_pred_label_file_list;
        int gt_label_idx = 0;
        int pred_label_idx = 0;
        while(gt_label_idx < gt_label_file_list.size() && pred_label_idx < pred_label_file_list.size() && ros::ok())
        {
            int gt_label_file_id = extract_number(gt_label_file_list[gt_label_idx]);
            int pred_label_file_id = extract_number(pred_label_file_list[pred_label_idx]);

            if(gt_label_file_id < pred_label_file_id)
                gt_label_idx++;
            else if(gt_label_file_id > pred_label_file_id)
                pred_label_idx++;
            else if(gt_label_file_id == pred_label_file_id)
            {
                tmp_gt_label_file_list.push_back(gt_label_file_list[gt_label_idx]);
                tmp_pred_label_file_list.push_back(pred_label_file_list[pred_label_file_id]);
                gt_label_idx++;
                pred_label_idx++;
            }
        }
        gt_label_file_list = tmp_gt_label_file_list;
        pred_label_file_list = tmp_pred_label_file_list;
    }

    // Limit frames
    if(i_start_frame > 0 || i_end_frame > 0)
    {
        int limit_start_frame = 0;
        if(i_start_frame > 0 && i_start_frame < gt_label_file_list.size())
            limit_start_frame = i_start_frame;
        int limit_end_frame = gt_label_file_list.size() - 1;
        if(i_end_frame > 0 && i_end_frame < gt_label_file_list.size())
            limit_end_frame = i_end_frame;

        std::vector<std::string> tmp_gt_label_file_list;
        std::vector<std::string> tmp_pred_label_file_list;
        for(int idx = 0; idx < gt_label_file_list.size(); idx++)
        {
            int gt_label_file_id = extract_number(gt_label_file_list[idx]);
            if(gt_label_file_id >= limit_start_frame && gt_label_file_id <= limit_end_frame)
                tmp_gt_label_file_list.push_back(gt_label_file_list[idx]);
            int pred_label_file_id = extract_number(pred_label_file_list[idx]);
            if(pred_label_file_id >= limit_start_frame && pred_label_file_id <= limit_end_frame)
                tmp_pred_label_file_list.push_back(pred_label_file_list[idx]);
        }
        gt_label_file_list = tmp_gt_label_file_list;
        pred_label_file_list = tmp_pred_label_file_list;
        std::cout << "Limit frames. num of selected frame: " << gt_label_file_list.size() << "\n";
    }

    if(gt_label_file_list.size() <= 0 || pred_label_file_list.size() <= 0)
    {
        std::cout << "Invalid data. gt label list size: " << gt_label_file_list.size() << ", pred label list size: " << pred_label_file_list.size() << "\n";
        return false;
    }

    // - Read label files
    std::vector<std::vector<int>> gt_labels;
    std::vector<std::vector<int>> pred_labels;
    for(int idx = 0; idx < gt_label_file_list.size() && ros::ok(); idx++)
    {
        std::string gt_label_folder_path = gt_label_file_list[idx];
        std::string pred_label_folder_path = pred_label_file_list[idx];

        uint32_t label;
        std::ifstream gt_label_file(gt_label_folder_path, std::ios::binary);
        std::ifstream pred_label_file(pred_label_folder_path, std::ios::binary);
        if (!gt_label_file.is_open()) 
        {
            std::cerr << "Failed to open gt label file: " << gt_label_folder_path << std::endl;
            return false;
        }
        if (!pred_label_file.is_open()) 
        {
            std::cerr << "Failed to open pred label file: " << pred_label_folder_path << std::endl;
            return false;
        }

        std::vector<int> gt_label;
        while (gt_label_file.read(reinterpret_cast<char*>(&label), sizeof(uint32_t))) 
        {
            uint16_t sem_label = label & 0xFFFF;
            uint16_t inst_label = label >> 16;
            gt_label.push_back((int)sem_label);
        }
        gt_label_file.close();

        std::vector<int> pred_label;
        while (pred_label_file.read(reinterpret_cast<char*>(&label), sizeof(uint32_t))) 
        {
            uint16_t sem_label = label & 0xFFFF;
            uint16_t inst_label = label >> 16;
            pred_label.push_back((int)sem_label);
        }
        pred_label_file.close();

        gt_labels.push_back(gt_label);
        pred_labels.push_back(pred_label);

        float progress = static_cast<float>(idx+1) / gt_label_file_list.size() * 100.0f;
        if((int)(progress * 1000) % 10 == 0)
            std::cout << "\rRead labels from kitti format Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                    << progress << "% (" << (idx+1) << "/" << gt_label_file_list.size() << ")" << std::flush;
    }
    std::cout << "\n";

    o_gt_labels = gt_labels;
    o_pred_labels = pred_labels;

    return true;
}

void EvaluateMOS(const std::vector<std::vector<int>>& i_gt_labels, const std::vector<std::vector<int>>& i_pred_labels, std::unordered_map<std::string, std::vector<std::pair<int, float>>>& o_eval_dict)
{
    /* 
    Index List
    - 0~1 : not evaluate
    - 9~99 : static
    - 251~259 : moving
    */

    std::unordered_map<std::string, std::vector<std::pair<int, float>>> eval_dict;
    long total_TP = 0; 
    long total_TN = 0; 
    long total_FP = 0; 
    long total_FN = 0; 
    
    int num_of_scans = i_gt_labels.size();
    for(int frame_idx = 0; frame_idx < num_of_scans; frame_idx++)
    {
        int num_of_points = i_gt_labels[frame_idx].size();
        int scan_TP = 0;
        int scan_TN = 0;
        int scan_FP = 0;
        int scan_FN = 0;

        for(int point_idx = 0; point_idx < num_of_points; point_idx++)
        {
            int gt_label = i_gt_labels[frame_idx][point_idx];
            int pred_label = i_pred_labels[frame_idx][point_idx];

            bool gt_is_dynamic;
            if(gt_label >= 9 && gt_label <= 99) // static
                gt_is_dynamic = false;
            else if(gt_label >= 251 && gt_label <= 259) // dynamic
                gt_is_dynamic = true;
            else
                continue;

            bool pred_is_dynamic;
            if(pred_label == 9) // static
                pred_is_dynamic = false;
            else if(pred_label == 251) // dynamic
                pred_is_dynamic = true;
            else
                continue;

            if(gt_is_dynamic == true && pred_is_dynamic == true)
                scan_TP++;
            else if(gt_is_dynamic == false && pred_is_dynamic == false)
                scan_TN++;
            else if(gt_is_dynamic == true && pred_is_dynamic == false)
                scan_FN++;
            else if(gt_is_dynamic == false && pred_is_dynamic == true)
                scan_FP++;
        }

        total_TP += scan_TP;
        total_TN += scan_TN;
        total_FP += scan_FP;
        total_FN += scan_FN;

        float scan_IoU = (float) (scan_TP + 1) / (float) (scan_TP + scan_FP + scan_FN + 1);
        float scan_accuracy = (float) (scan_TP + scan_TN + 1) / (float) (scan_TP + scan_TN + scan_FP + scan_FN + 1);
        float scan_precision = (float) (scan_TP + 1) / (float) (scan_TP + scan_FP + 1);
        float scan_recall = (float) (scan_TP + 1) / (float) (scan_TP + scan_FN + 1);
        float scan_PR = (float) (scan_TN + 1) / (float) (scan_TN + scan_FP + 1);
        float scan_RR = (float) (scan_TP + 1) / (float) (scan_TP + scan_FN + 1);
        float scan_score = (scan_PR + scan_RR) / 2.0; 

        o_eval_dict["scan_IoU"].emplace_back(frame_idx, scan_IoU);
        o_eval_dict["scan_accuracy"].emplace_back(frame_idx, scan_accuracy);
        o_eval_dict["scan_precision"].emplace_back(frame_idx, scan_precision);
        o_eval_dict["scan_recall"].emplace_back(frame_idx, scan_recall);
        o_eval_dict["scan_PR"].emplace_back(frame_idx, scan_PR);
        o_eval_dict["scan_RR"].emplace_back(frame_idx, scan_RR);
        o_eval_dict["scan_score"].emplace_back(frame_idx, scan_score);
        o_eval_dict["scan_TP"].emplace_back(frame_idx, scan_TP);
        o_eval_dict["scan_TN"].emplace_back(frame_idx, scan_TN);
        o_eval_dict["scan_FP"].emplace_back(frame_idx, scan_FP);
        o_eval_dict["scan_FN"].emplace_back(frame_idx, scan_FN);

        float progress = static_cast<float>(frame_idx+1) / num_of_scans * 100.0f;
        if((int)(progress * 1000) % 10 == 0)
        {
            std::cout << "\rEvaluation Progress: " << std::setw(6) << std::fixed << std::setprecision(1)
                    << progress << "% (" << (frame_idx + 1) << "/" << num_of_scans << ")" << std::flush;
        }
    }
    std::cout << "\n";

    o_eval_dict["total_TP"].emplace_back(0, total_TP);
    o_eval_dict["total_TN"].emplace_back(0, total_TN);
    o_eval_dict["total_FP"].emplace_back(0, total_FP);
    o_eval_dict["total_FN"].emplace_back(0, total_FN);

    return;
}

bool WriteEvaluationResult(const std::string i_write_result_path, const std::unordered_map<std::string, std::vector<std::pair<int, float>>>& i_eval_dict)
{
    
    int total_TP = (int)i_eval_dict.at("total_TP").back().second;
    int total_TN = (int)i_eval_dict.at("total_TN").back().second;
    int total_FP = (int)i_eval_dict.at("total_FP").back().second;
    int total_FN = (int)i_eval_dict.at("total_FN").back().second;
    float total_IoU = (float) total_TP / (float) (total_TP + total_FP + total_FN + FLT_MIN);
    float total_accuracy = (float) (total_TP + total_TN) / (float) (total_TP + total_TN + total_FP + total_FN + FLT_MIN);
    float total_precision = (float) (total_TP) / (float) (total_TP + total_FP + FLT_MIN);
    float total_recall = (float) (total_TP) / (float) (total_TP + total_FN + FLT_MIN);
    float total_PR = (float) total_TN / (float) (total_TN + total_FP + FLT_MIN);
    float total_RR = (float) total_TP / (float) (total_TP + total_FN + FLT_MIN);
    float total_score = (total_PR + total_RR) / 2.0; 

    std::ofstream file_output(i_write_result_path, std::ios::out);
    if (file_output.is_open() == false)
    {
        std::cout << "fail to open result file. Path : " << i_write_result_path << "\n";
        return false;
    }

    file_output.setf(std::ios::fixed);
    file_output.precision(6);
    file_output << "scan_id,scan_IoU,scan_accuracy,scan_precision,scan_recall,scan_PR,scan_RR,scan_score,scan_TP,scan_TN,scan_FP,scan_FN,total_IoU,total_accuracy,total_precision,total_recall,total_PR,total_RR,total_score,total_TP,total_TN,total_FP,total_FN\n";

    for(int i = 0; i < (int)i_eval_dict.at("scan_IoU").size(); i++)
    {
        if(i == 0)
        {
            file_output 
                << i_eval_dict.at("scan_IoU")[i].first << "," 
                << i_eval_dict.at("scan_IoU")[i].second << "," 
                << i_eval_dict.at("scan_accuracy")[i].second << "," 
                << i_eval_dict.at("scan_precision")[i].second << "," 
                << i_eval_dict.at("scan_recall")[i].second << "," 
                << i_eval_dict.at("scan_PR")[i].second << "," 
                << i_eval_dict.at("scan_RR")[i].second << "," 
                << i_eval_dict.at("scan_score")[i].second << "," 
                << i_eval_dict.at("scan_TP")[i].second << "," 
                << i_eval_dict.at("scan_TN")[i].second << "," 
                << i_eval_dict.at("scan_FP")[i].second << "," 
                << i_eval_dict.at("scan_FN")[i].second << "," 
                << total_IoU << "," << total_accuracy << "," << total_precision << "," 
                << total_recall << "," << total_PR << "," << total_RR << "," 
                << total_score << "," << total_TP << "," << total_TN << "," 
                << total_FP << "," << total_FN << "\n";
        }
        else
        {
            file_output 
                << i_eval_dict.at("scan_IoU")[i].first << "," 
                << i_eval_dict.at("scan_IoU")[i].second << "," 
                << i_eval_dict.at("scan_accuracy")[i].second << "," 
                << i_eval_dict.at("scan_precision")[i].second << "," 
                << i_eval_dict.at("scan_recall")[i].second << "," 
                << i_eval_dict.at("scan_PR")[i].second << "," 
                << i_eval_dict.at("scan_RR")[i].second << "," 
                << i_eval_dict.at("scan_score")[i].second << "," 
                << i_eval_dict.at("scan_TP")[i].second << "," 
                << i_eval_dict.at("scan_TN")[i].second << "," 
                << i_eval_dict.at("scan_FP")[i].second << "," 
                << i_eval_dict.at("scan_FN")[i].second << "\n";
        }
    }
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::string(40, '-') << "\n";
    std::cout << "Total IoU: " << total_IoU << ", precision: " << total_precision << ", recall: " << total_recall 
              << ", TP: " << total_TP << ", TN: " << total_TN << ", FP: " << total_FP << ", FN: " << total_FN << "\n";
    std::cout << "Evaluation result write done. path: " << i_write_result_path << "\n";
    std::cout << std::string(40, '-') << "\n";

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mos_evaluation");

    // Get params
    ros::NodeHandle nhPrivate("~");
    bool use_bag_file, use_label_file;
    int start_frame, end_frame;
    std::string bag_file_path, scan_topic, gt_label_folder_path, pred_label_folder_path, result_folder_path;
    if(!nhPrivate.getParam("/mos_evaluation/use_bag_file", use_bag_file)){{ROS_WARN("Fail to get param - use_bag_file");}};
    if(!nhPrivate.getParam("/mos_evaluation/bag_file_path", bag_file_path)){{ROS_WARN("Fail to get param - bag_file_path");}};
    if(!nhPrivate.getParam("/mos_evaluation/use_label_file", use_label_file)){{ROS_WARN("Fail to get param - use_label_file");}};
    if(!nhPrivate.getParam("/mos_evaluation/gt_label_folder_path", gt_label_folder_path)){{ROS_WARN("Fail to get param - gt_label_folder_path");}};
    if (!gt_label_folder_path.empty() && gt_label_folder_path.back() != '/')
        gt_label_folder_path += '/';
    if(!nhPrivate.getParam("/mos_evaluation/pred_label_folder_path", pred_label_folder_path)){{ROS_WARN("Fail to get param - pred_label_folder_path");}};
    if (!pred_label_folder_path.empty() && pred_label_folder_path.back() != '/')
        pred_label_folder_path += '/';
    if(!nhPrivate.getParam("/mos_evaluation/start_frame", start_frame)){{ROS_WARN("Fail to get param - start_frame");}};
    if(!nhPrivate.getParam("/mos_evaluation/end_frame", end_frame)){{ROS_WARN("Fail to get param - end_frame");}};
    if(!nhPrivate.getParam("/mos_evaluation/scan_topic", scan_topic)){{ROS_WARN("Fail to get param - scan_topic");}};
    if(!nhPrivate.getParam("/mos_evaluation/result_folder_path", result_folder_path)){{ROS_WARN("Fail to get param - result_folder_path");}};
    if (!result_folder_path.empty() && result_folder_path.back() != '/')
        result_folder_path += '/';

    // Set dictionary to write results
    std::unordered_map<std::string, std::vector<std::pair<int, float>>> eval_dict;
    eval_dict["scan_IoU"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_accuracy"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_precision"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_recall"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_PR"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_RR"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_score"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_TP"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_TN"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_FP"] = std::vector<std::pair<int, float>>();
    eval_dict["scan_FN"] = std::vector<std::pair<int, float>>();
    eval_dict["total_TP"] = std::vector<std::pair<int, float>>();
    eval_dict["total_TN"] = std::vector<std::pair<int, float>>();
    eval_dict["total_FP"] = std::vector<std::pair<int, float>>();
    eval_dict["total_FN"] = std::vector<std::pair<int, float>>();


    // Get Labels
    std::vector<std::vector<int>> gt_labels;
    std::vector<std::vector<int>> pred_labels;
    if(use_bag_file == true)
    {
        if(GetLabelsFromBag(bag_file_path, scan_topic, start_frame, end_frame, gt_labels, pred_labels) == false) {return -1;};
    }
    else if(use_label_file == true)
    {
        if(GetLabelsFromKitti(gt_label_folder_path, pred_label_folder_path, start_frame, end_frame, gt_labels, pred_labels) == false) {return -1;};
    }
    else
    {
        std::cout << "Need to set evaluation source. ('use_bag_file' or 'use_label_file')\n";
        return -1;
    }

    if(gt_labels.size() <= 0 || pred_labels.size() <= 0)
    {
        std::cout << "Invalid data. gt labels size: " << gt_labels.size() << ", pred labels size: " << pred_labels.size() << "\n";
        return -1;
    }

    // Run evaluation
    EvaluateMOS(gt_labels, pred_labels, eval_dict);

    // Write results
    std::string result_file_path;
    if(use_bag_file == true)
    {
        size_t last_slash_idx = bag_file_path.find_last_of("/\\");
        size_t expender_idx = bag_file_path.find_last_of(".");
        std::string bag_file_name;
        if (last_slash_idx != std::string::npos && expender_idx != std::string::npos)
            bag_file_name = bag_file_path.substr(last_slash_idx + 1, expender_idx - last_slash_idx - 1);


        std::string result_file_name = bag_file_name + "_mos_evaluation.csv";
        result_file_path = result_folder_path + result_file_name;
    }
    else
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S_");

        // Write config file log
        std::string result_file_name = ss.str() + "mos_evaluation.csv";;
        result_file_path = result_folder_path + result_file_name;
    }

    WriteEvaluationResult(result_file_path, eval_dict);

    return 0;
}
