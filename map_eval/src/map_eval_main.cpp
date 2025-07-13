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
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;

    std::string config_file = "../config/config.yaml";
    Param param =  loadParametersFromYAML(config_file);

    MapEval map_eval(param);
    map_eval.process();

    return EXIT_SUCCESS;
}
