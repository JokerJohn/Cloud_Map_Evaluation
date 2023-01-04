#include "cloud_map_eval.h"

/**
 * for map evaluation, we have several metrics
 * 1. accuracy p: rmse mean_value
 * 2. completeness r: ovelap ratio
 * 3. F1 score: 2pr/(p+r)
 */

int main(int argc, char **argv) {
  bool save_immediate_result = true;

  // evaluation method, 0: point-to-point icp 1: point-to-plane icp 2: GICP
  int method = 2;
  // max correspondence pairs distance for icp knn search corresspondence
  // iteration(the search radius for kdtree)
  double icp_max_distance = 0.2;

  // set evaluatation accucay level, do not recommand to change, eg. 20cm/10cm/5cm/2cm/1cm
  Vector5d accuacy_level = Vector5d::Zero();
  accuacy_level << 0.2, 0.1, 0.05, 0.02, 0.01;

  // for map produced by LIO, we need a initial pose
  // we can use cloud compare to align your maps to get the initial pose
  Eigen::Matrix4d initial_matrix = Eigen::Matrix4d::Identity();

  // the path dir must end with '/'
  std::string est_path, gt_path, results_path, sequence_name;
  sequence_name = "MCR_slow";
  est_path = "../dataset/MCR_slow/";
  gt_path = "../dataset/MCR_slow/MCR_gt.pcd";
  results_path = "../dataset/MCR_slow/";

  //  MCR_slow_00
  //  initial_matrix << 0.076328999, -0.997022781, -0.010945004, -3.413845980,
  //      0.997082425, 0.076330289, 0.000297473, 3.706331567, 0.000538848,
  //      -0.010935786, 0.999940157, -0.408766363, 0.000000000, 0.000000000,
  //      0.000000000, 1.000000000;

  // params
  Param my_param(est_path, gt_path, results_path, initial_matrix, sequence_name,
                 method, icp_max_distance, accuacy_level,
                 save_immediate_result);

  CloudMapEva my_evaluation(my_param );
  my_evaluation.process();

  return EXIT_SUCCESS;
}
