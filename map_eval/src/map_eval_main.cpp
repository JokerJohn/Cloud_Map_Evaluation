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

int main(int argc, char **argv) {
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;

    // load all the yaml parameters
    YAML::Node config = YAML::LoadFile("../config/config.yaml");
    int method = config["registration_methods"].as<int>();
    double icp_max_distance = config["icp_max_distance"].as<double>();

    Vector5d accuracy_level = Vector5d::Zero();
    for (int i = 0; i < 5; ++i) {
        accuracy_level[i] = config["accuracy_level"][i].as<double>();
    }
    Eigen::Matrix4d initial_matrix = Eigen::Matrix4d::Zero();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            initial_matrix(i, j) = config["initial_matrix"][i][j].as<double>();
        }
    }
    // load all the settings
    bool save_immediate_result = config["save_immediate_result"].as<bool>();
    bool evaluate_mme = config["evaluate_mme"].as<bool>();
    bool evaluate_gt_mme = config["evaluate_gt_mme"].as<bool>();
    bool evaluate_using_initial_ = config["evaluate_using_initial"].as<bool>();
    bool evaluate_noise_gt_ = config["evaluate_noise_gt"].as<bool>();
    double nn_radius = config["nn_radius"].as<double>();
    double vmd_voxel_size = config["vmd_voxel_size"].as<double>();

    // the path dir must end with '/'
    std::string est_path, gt_path, results_path, scene_name;
    est_path = config["estimate_map_path"].as<std::string>();
    gt_path = config["gt_map_path"].as<std::string>();
    scene_name = config["scene_name"].as<std::string>();
    results_path = est_path + "map_results/";
    std::string pcd_file_name = "map.pcd";

    Param param(est_path, gt_path, results_path, initial_matrix, scene_name, method, icp_max_distance, accuracy_level, 
         save_immediate_result, evaluate_mme, evaluate_gt_mme, pcd_file_name, evaluate_using_initial_,  evaluate_noise_gt_, 0.01, nn_radius, vmd_voxel_size);
    MapEval map_eval(param);
    map_eval.process();

    return EXIT_SUCCESS;
}
