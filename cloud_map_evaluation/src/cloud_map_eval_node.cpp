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
    double icp_max_distance = 0.5;

    // set evaluation accuracy level, do not recommend to change, eg. 20cm/10cm/5cm/2cm/1cm
    Vector5d accuracy_level = Vector5d::Zero();
    accuracy_level << 0.2, 0.1, 0.05, 0.02, 0.01;

    // for map produced by LIO, we need a initial pose
    // we can use cloud compare to align your maps to get the initial pose
    Eigen::Matrix4d initial_matrix = Eigen::Matrix4d::Identity();

    //  MCR_SLOW
    //      initial_matrix << 0.091375982 ,0.995817066,-0.006855756,-5.357079965,
    //            -0.995934835,0.091492701 ,0.008818712,5.485868708,
    //            0.010135263 ,0.005883287,0.999801271,  0.579084110,
    //            0.000000000,0.000000000 ,0.000000000,  1.00000000;

    // the path dir must end with '/'
    std::string est_path, gt_path, results_path, sequence_name;
    std::string est_folder = "/home/xchu/my_git/Cloud_Map_Evaluation/cloud_map_evaluation/dataset/";
    sequence_name = "MCR_slow";
    est_path = est_folder + sequence_name + "/";
    gt_path = est_folder + sequence_name + "/" + sequence_name + "_gt.pcd";
    results_path = est_folder + sequence_name + "/";

    Param my_param(est_path, gt_path, results_path, initial_matrix, sequence_name,
                   method, icp_max_distance, accuracy_level,
                   save_immediate_result);
    CloudMapEva my_evaluation(my_param);
    my_evaluation.process();

    return EXIT_SUCCESS;
}
