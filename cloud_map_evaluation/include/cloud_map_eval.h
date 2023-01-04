/*******************************************************
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of FL2SAM (https://github.com/JokerJohn/FL2SAM-GPS).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Xiangcheng HU, Jianhao Jiao, Tianshuai HU
 *******************************************************/
#ifndef SRC_POSE_SLAM_PRIOR_SRC_BENCHMARK_CLOUD_MAP_EVAL_H_
#define SRC_POSE_SLAM_PRIOR_SRC_BENCHMARK_CLOUD_MAP_EVAL_H_
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tic_toc.h"

using namespace std;
using namespace Eigen;
using namespace open3d;
typedef geometry::PointCloud PointCloud;
typedef geometry::TriangleMesh Mesh;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<int, 5, 1> Vector5i;

struct Param {
  std::string evaluation_map_pcd_path_ = "/data/map_evaluation/canteen/";
  std::string map_gt_path_ = "/data/map_evaluation/canteen/merged_scan.pcd";
  std::string result_path_ = "/home/hts/workspace/dataset/eva_results/";

  std::string name_;
  int evaluation_method_ = 2;
  // int voxel_dim_ = 2;
  double voxel_size_ = 1.0;
  double icp_max_distance_ = 2.5;
  // double colorbar_max_unit_ = 2;

  bool save_immediate_result_ = false;
  Vector5d trunc_dist_;
  // double outlier_dist_;
  Eigen::Matrix4d initial_matrix_ = Eigen::Matrix4d::Identity();

  Param() {}
  Param(std::string evaluation_map_pcd_path, std::string map_gt_path,
        std::string result_path, Eigen::Matrix4d initial_matrix,
        std::string name, int method, double icp_max_distance,
        Vector5d trunc_dist, bool save_immediate_result)
      : evaluation_map_pcd_path_(evaluation_map_pcd_path),
        map_gt_path_(map_gt_path),
        result_path_(result_path),
        initial_matrix_(initial_matrix),
        name_(name),
        evaluation_method_(method),
        // voxel_dim_(dim),
        icp_max_distance_(icp_max_distance),
        trunc_dist_(trunc_dist),
        // colorbar_max_unit_(colorbar_max_unit),
        save_immediate_result_(save_immediate_result) {}

  void printParam() {
    std::cout
        << "\n[===================="
        << " name: " << name_ << "=====================]"
        << "\n evaluation_map_pcd_path: " << evaluation_map_pcd_path_
        << "\n map_gt_path: " << map_gt_path_
        << "\n result_path: " << result_path_ << "\n initial matrix: \n "
        << initial_matrix_ << "\n icp_max_distance: " << icp_max_distance_
        << "\n evaluation_method_: "
        << evaluation_method_
        //        << "\n colorbar_max_unit: " << colorbar_max_unit_
        << "\n trunc_dist: " << trunc_dist_.transpose()
        << "\n save_immediate_result: " << save_immediate_result_
        << "\n "
           "===========================================================\n ";
  }
};


class CloudMapEva {
 public:
  CloudMapEva(Param &param)
      : param_(param), map_3d_(new PointCloud), gt_3d_(new PointCloud) {
    param_.printParam();
  }

  int process();

  int loadTrajTUM(const string &traj_path, vector<Vector8d> &traj_vec);
  int saveTrajTUM(const string &traj_path, vector<Vector8d> &traj_vec);

  template<class T>
  int calculateRMSE(vector<T> &result_vec);

  template<typename T>
  map<string, double> calculateError(vector<T> &result_vec);

  int createGridMap();

  /**
   *
   * @return {int}  : remove points from the est_map if they do not stay at the
   * region of gt_map. This function ensure that the the coverage of the gt_map
   * should be always larger than the est_map.
   */
  int crop(shared_ptr<PointCloud> &reference_points,
           shared_ptr<PointCloud> &target_points);

  vector<double> computePointCloudDistance(
      shared_ptr<PointCloud> &reference_points,
      shared_ptr<PointCloud> &target_points);

  void computeDifferentMetrics(double thre, double mean);

  double evaluateChamferDistance(std::shared_ptr<PointCloud> &reference_points,
                                 std::shared_ptr<PointCloud> &target_points);

  std::shared_ptr<PointCloud> renderDistanceOnPointCloud(
      std::shared_ptr<PointCloud> &reference_points,
      std::shared_ptr<PointCloud> &target_points, const double &dis);

  std::shared_ptr<Mesh> renderDistanceOnMesh(
      std::shared_ptr<PointCloud> &reference_points,
      std::shared_ptr<Mesh> &target_mesh,
      std::shared_ptr<PointCloud> &target_points, double dis);

  shared_ptr<Mesh> createMeshFromPCD(shared_ptr<PointCloud> &reference_points,
                                     double density_thres, int depth);

  void getDiffRegResult(std::vector<Vector5d> &result,
                        pipelines::registration::CorrespondenceSet &points_set,
                        geometry::PointCloud &source,
                        geometry::PointCloud &target);

  void getDiffRegResult(std::vector<Vector5d> &result,
                        pipelines::registration::CorrespondenceSet &points_set,
                        geometry::PointCloud &source,
                        geometry::PointCloud &target,
                        Eigen::MatrixXd &source_set,
                        Eigen::MatrixXd &target_set);


 public:
  Param param_;

 private:
  shared_ptr<PointCloud> map_3d_, gt_3d_;
 //  shared_ptr<open3d::geometry::VoxelGrid> voxel_grid_gt_;
};

#endif// SRC_POSE_SLAM_PRIOR_SRC_BENCHMARK_CLOUD_MAP_EVAL_H_
