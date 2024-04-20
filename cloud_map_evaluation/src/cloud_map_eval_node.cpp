#include "cloud_map_eval.h"
#include <filesystem>


/**
 * for map evaluation, we have several metrics
 * 1. accuracy p: rmse mean_value
 * 2. completeness r: overlap ratio
 * 3. F1 score: 2pr/(p+r)
 */
int main(int argc, char **argv) {
    bool save_immediate_result = true;
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;

    // evaluation method, 0: point-to-point icp 1: point-to-plane icp 2: GICP
    int method = 2;
    // max correspondence pairs distance for icp knn search correspondence
    // iteration(the search radius for kdtree)
    //    double icp_max_distance = 1.0;  for large scale outdoor map
    double icp_max_distance = 0.5;

    // set evaluation accuracy level, do not recommend to change, eg. 20cm/10cm/5cm/2cm/1cm
    Vector5d accuracy_level = Vector5d::Zero();
    accuracy_level << 0.2, 0.1, 0.05, 0.02, 0.01;
    //    accuracy_level << 0.5, 0.3, 0.1, 0.05, 0.03;

    // for map produced by LIO, we need a initial pose
    // we can use cloud compare to align your maps to get the initial pose
    Eigen::Matrix4d initial_matrix = Eigen::Matrix4d::Identity();

    // the path dir must end with '/'
    std::string est_path, gt_path, results_path, sequence_name;
    std::string est_folder = "/home/xchu/my_git/Cloud_Map_Evaluation/cloud_map_evaluation/dataset/";
    sequence_name = "MCR_slow";
    est_path = est_folder + sequence_name + "/";
    gt_path = est_folder + sequence_name + "/" + sequence_name + "_gt.pcd";
    results_path = est_folder + sequence_name + "/";

    // if you evaluate mme
    bool evaluate_mme = false;
    bool evaluate_gt_mme = false;
    Param my_param(est_path, gt_path, results_path, initial_matrix, sequence_name,
                   method, icp_max_distance, accuracy_level,
                   save_immediate_result, evaluate_mme, evaluate_gt_mme);
    CloudMapEva my_evaluation(my_param);
    my_evaluation.process();

    return EXIT_SUCCESS;
}
