/**
*  @file   map_eval_node.cpp
*  @brief  Evaluate the accuracy of Point Cloud Maps using various metrics (e.g., Chamfer Distance, F1 Score, IoU).
*  @author Xiangcheng Hu (xhubd@connect.ust.hk)
*  @date   Feb 12, 2025
*  
*  The code is part of the MapEval project, which aims to provide a unified, robust, and efficient evaluation framework for SLAM maps.
*  The code is open-source and can be freely used for research purposes.
*  If you use this code in your research, please consider citing the following paper:
*  MapEval: Towards Unified, Robust and Efficient SLAM Map Evaluation Framework (https://arxiv.org/abs/2411.17928)
* 
*  In a normal scenario, we recommend using the AC, CD, or AWD metrics to evaluate the accuracy of the SLAM maps.
*  But when you do not have the ground truth map, you can only use the MME.
*
*  We appreciate any contributions to the project, and you can pull requests to the GitHub repository. 
*  For more information about the MapEval project, please visit the project website: https://github.com/JokerJohn/Cloud_Map_Evaluation
*  Copyright (C) 2025, Hong Kong University of Science and Technology.
*/

#include "map_eval.h"
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <iomanip>
#include <sstream>

// Display all program information in one comprehensive function
void displayProgramInformation(const Param& param) {
    // Program Header
    std::cout << "\n";
    std::cout << "================================================================================\n";
    std::cout << "                              MapEval v1.0.0                                    \n";
    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << "    Unified, Robust and Efficient SLAM Map Evaluation Framework                 \n";
    std::cout << "    https://github.com/JokerJohn/Cloud_Map_Evaluation                          \n";
    std::cout << "================================================================================\n\n";
    
    // Configuration Parameters
    std::cout << "╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                          EVALUATION CONFIGURATION                              ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════════════════╣\n";
    
    // Scene Information
    std::cout << "║ Scene Information:                                                             ║\n";
    std::cout << "║ ├─ Scene Name: " << std::left << std::setw(61) << param.name_ << " ║\n";
    std::cout << "║ ├─ GT Map: " << std::left << std::setw(65) 
              << (param.map_gt_path_.length() > 65 ? "..." + param.map_gt_path_.substr(param.map_gt_path_.length() - 62) : param.map_gt_path_) 
              << " ║\n";
    std::cout << "║ └─ Est Map: " << std::left << std::setw(64) 
              << (param.evaluation_map_pcd_path_.length() > 64 ? "..." + param.evaluation_map_pcd_path_.substr(param.evaluation_map_pcd_path_.length() - 61) : param.evaluation_map_pcd_path_) 
              << " ║\n";
    
    std::cout << "╠══════════════════════════════════════════════════════════════════════════════╣\n";
    
    // Registration Settings
    std::cout << "║ Registration Settings:                                                         ║\n";
    std::string reg_method = (param.evaluation_method_ == 0) ? "Point-to-Point ICP" : 
                            (param.evaluation_method_ == 1) ? "Point-to-Plane ICP" : "GICP";
    std::cout << "║ ├─ Method: " << std::left << std::setw(65) << reg_method << " ║\n";
    std::cout << "║ ├─ Max Distance: " << std::left << std::setw(59) << param.icp_max_distance_ << " ║\n";
    std::cout << "║ ├─ Downsample Size: " << std::left << std::setw(56) << param.downsample_size << " ║\n";
    std::cout << "║ └─ Use Initial Matrix: " << std::left << std::setw(53) 
              << (param.evaluate_using_initial_ ? "Yes" : "No") << " ║\n";
    
    std::cout << "╠══════════════════════════════════════════════════════════════════════════════╣\n";
    
    // Evaluation Metrics
    std::cout << "║ Evaluation Metrics:                                                            ║\n";
    std::cout << "║ ├─ Evaluate MME: " << std::left << std::setw(59) 
              << (param.evaluate_mme_ ? "Enabled" : "Disabled") << " ║\n";
    if (param.evaluate_mme_) {
        std::cout << "║ │  ├─ GT MME: " << std::left << std::setw(62) 
                  << (param.evaluate_gt_mme_ ? "Enabled" : "Disabled") << " ║\n";
        std::cout << "║ │  ├─ NN Radius: " << std::left << std::setw(59) << param.nn_radius_ << " ║\n";
        std::cout << "║ │  └─ TBB Acceleration: " << std::left << std::setw(52) 
                  << (param.use_tbb_mme ? "Enabled" : "Disabled") << " ║\n";
    }
    std::cout << "║ ├─ AWD Voxel Size: " << std::left << std::setw(57) << param.vmd_voxel_size_ << " ║\n";
    std::cout << "║ └─ Accuracy Levels: ";
    std::stringstream acc_stream;
    acc_stream << std::fixed << std::setprecision(2) << "[";
    for (int i = 0; i < 5; ++i) {
        acc_stream << param.trunc_dist_[i];
        if (i < 4) acc_stream << ", ";
    }
    acc_stream << "]";
    std::cout << std::left << std::setw(56) << acc_stream.str() << " ║\n";
    
    std::cout << "╠══════════════════════════════════════════════════════════════════════════════╣\n";
    
    // Output Settings
    std::cout << "║ Output Settings:                                                               ║\n";
    std::cout << "║ ├─ Save Results: " << std::left << std::setw(59) 
              << (param.save_immediate_result_ ? "Enabled" : "Disabled") << " ║\n";
    std::cout << "║ ├─ Visualization: " << std::left << std::setw(58) 
              << (param.use_visualization ? "Enabled" : "Disabled") << " ║\n";
    std::cout << "║ └─ Debug Mode: " << std::left << std::setw(61) 
              << (param.enable_debug ? "Enabled" : "Disabled") << " ║\n";
    
    std::cout << "╠══════════════════════════════════════════════════════════════════════════════╣\n";
    
    // System Information
    std::cout << "║ System Information:                                                            ║\n";
    std::cout << "║ ├─ CPU Threads: " << std::left << std::setw(60) << std::thread::hardware_concurrency() << " ║\n";
//    std::cout << "║ ├─ OpenMP: Enabled                                                             ║\n";
//    std::cout << "║ ├─ TBB: Enabled                                                                ║\n";
    std::cout << "║ └─ Working Dir: " << std::left << std::setw(60) 
              << (std::filesystem::current_path().string().length() > 60 ? "..." + std::filesystem::current_path().string().substr(std::filesystem::current_path().string().length() - 57) : std::filesystem::current_path().string())
              << " ║\n";
    
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n\n";
    
    // Citation Information
    std::cout << "Citation: If you use MapEval in your research, please cite:\n";
    std::cout << "\"MapEval: Towards Unified, Robust and Efficient SLAM Map Evaluation Framework\"\n";
    std::cout << "Paper: https://ieeexplore.ieee.org/document/10910156\n\n";
}

// 从YAML文件加载参数的函数
Param loadParametersFromYAML(const std::string& yaml_file_path) {
    try {
        // 加载YAML配置文件
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        // 创建默认的Param对象
        Param param;

        // 设置基本参数
        param.evaluation_method_ = config["registration_methods"].as<int>();
        param.icp_max_distance_ = config["icp_max_distance"].as<double>();

        // 设置精度级别向量
        if (config["accuracy_level"] && config["accuracy_level"].size() >= 5) {
            for (int i = 0; i < 5; ++i) {
                param.trunc_dist_[i] = config["accuracy_level"][i].as<double>();
            }
        }

        // 设置初始变换矩阵
        if (config["initial_matrix"] && config["initial_matrix"].size() >= 4) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    param.initial_matrix_(i, j) = config["initial_matrix"][i][j].as<double>();
                }
            }
        }

        // 设置布尔参数
        param.save_immediate_result_ = config["save_immediate_result"].as<bool>();
        param.evaluate_mme_ = config["evaluate_mme"].as<bool>();
        param.evaluate_gt_mme_ = config["evaluate_gt_mme"].as<bool>();
        param.evaluate_using_initial_ = config["evaluate_using_initial"].as<bool>();

        // 设置数值参数
        param.nn_radius_ = config["nn_radius"].as<double>();
        param.vmd_voxel_size_ = config["vmd_voxel_size"].as<double>();
        param.downsample_size = config["downsample_size"].as<double>();

        // 设置路径参数
        param.evaluation_map_pcd_path_ = config["estimate_map_path"].as<std::string>();
        param.map_gt_path_ = config["gt_map_path"].as<std::string>();
        param.name_ = config["scene_name"].as<std::string>();

        // 确保路径以'/'结尾
        if (!param.evaluation_map_pcd_path_.empty() && param.evaluation_map_pcd_path_.back() != '/') {
            param.evaluation_map_pcd_path_ += '/';
        }

        // 构建结果路径
        param.result_path_ = param.evaluation_map_pcd_path_ + "map_results/";

        // 设置可选参数（如果存在）
        if (config["pcd_file_name"]) {
            param.pcd_file_name_ = config["pcd_file_name"].as<std::string>();
        }

        if (config["evaluate_noised_gt"]) {
            param.evaluate_noised_gt_ = config["evaluate_noised_gt"].as<bool>();
        }

        if (config["noise_std_dev"]) {
            param.noise_std_dev_ = config["noise_std_dev"].as<double>();
        }

        if (config["voxel_size"]) {
            param.voxel_size_ = config["voxel_size"].as<double>();
        }

        if (config["use_visualization"]) {
            param.use_visualization = config["use_visualization"].as<bool>();
        }
        param.enable_debug = config["enable_debug"].as<bool>();
        
        if (config["use_tbb_mme"]) {
            param.use_tbb_mme = config["use_tbb_mme"].as<bool>();
        }


        return param;

    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        throw std::runtime_error("Failed to parse YAML file: " + yaml_file_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading parameters: " << e.what() << std::endl;
        throw;
    }
}


int main(int argc, char **argv) {
    // Set default config file or use command line argument
    std::string config_file = "../config/config_building_day.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }
    
    std::cout << "Loading configuration from: " << config_file << "\n";
    
    // Load parameters from YAML configuration
    Param param;
    try {
        param = loadParametersFromYAML(config_file);
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] Failed to load configuration: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
    
    // Display all program information in one place
    displayProgramInformation(param);
    
    std::cout << "Starting evaluation...\n";
    std::cout << "================================================================================\n\n";
    
    // Create and run evaluation
    MapEval map_eval(param);
    map_eval.process();

    std::cout << "\n================================================================================\n";
    std::cout << "Evaluation completed successfully!\n";
    std::cout << "================================================================================\n\n";

    return EXIT_SUCCESS;
}
