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
#include <filesystem>
#include <experimental/filesystem>
#include <future>
#include "tic_toc.h"
#include "voxel_calculator.hpp"


#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif


using namespace std;
using namespace Eigen;
using namespace open3d;
typedef geometry::PointCloud PointCloud;
typedef geometry::TriangleMesh Mesh;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<int, 5, 1> Vector5i;

#include <functional>

template<typename T>
struct hash_eigen {
    std::size_t operator()(const T &k) const {
        return std::hash<int>()(k[0]) ^ std::hash<int>()(k[1]) ^ std::hash<int>()(k[2]);
    }
};

struct Param {
    std::string evaluation_map_pcd_path_ = "/data/map_evaluation/canteen/";
    std::string map_gt_path_ = "/data/map_evaluation/canteen/merged_scan.pcd";
    std::string result_path_ = "/home/hts/workspace/dataset/eva_results/";
    std::string pcd_file_name_ = "map.pcd";

    std::string name_;
    int evaluation_method_ = 2;
    double voxel_size_ = 1.0;
    double icp_max_distance_ = 2.5;
    double nn_radius_ = 0.2;

    bool save_immediate_result_ = false;
    bool evaluate_mme_ = true;
    bool evaluate_gt_mme_ = true;
    bool evaluate_using_initial_ = true;
    bool evaluate_noised_gt_ = false;
    Vector5d trunc_dist_;
    Eigen::Matrix4d initial_matrix_ = Eigen::Matrix4d::Identity();
    double noise_std_dev_ = 0.1;
    double vmd_voxel_size_ = 3.0;

    Param() {}

    Param(std::string evaluation_map_pcd_path, std::string map_gt_path,
          std::string result_path, Eigen::Matrix4d initial_matrix,
          std::string name, int method, double icp_max_distance,
          Vector5d trunc_dist, bool save_immediate_result, bool evaluate_mme, bool evaluate_gt_mme,
          std::string pcd_file_name, bool evaluate_using_initial, bool evaluate_noised_gt, double noise_std_dev, double nn_radius,
           double vmd_voxel_size
        )
            : evaluation_map_pcd_path_(evaluation_map_pcd_path),
              map_gt_path_(map_gt_path),
              result_path_(result_path),
              initial_matrix_(initial_matrix),
              name_(name),
              evaluation_method_(method),
              icp_max_distance_(icp_max_distance),
              trunc_dist_(trunc_dist),
              save_immediate_result_(save_immediate_result),
              evaluate_mme_(evaluate_mme), evaluate_gt_mme_(evaluate_gt_mme),
              pcd_file_name_(pcd_file_name),
              evaluate_using_initial_(evaluate_using_initial),
              evaluate_noised_gt_(evaluate_noised_gt),
              noise_std_dev_(noise_std_dev),
              nn_radius_(nn_radius),
              vmd_voxel_size_(vmd_voxel_size)
              {}

    void printParam() {
        std::cout
                << "\n[===================="
                << " name: " << name_ << "=====================]"
                << "\n estmate map folder: " << evaluation_map_pcd_path_
                << "\n gt map path: " << map_gt_path_
                << "\n result_path: " << result_path_ << "\n initial matrix: \n "
                << initial_matrix_ << "\n icp_max_distance: " << icp_max_distance_
                << "\n evaluation_method_: "
                << evaluation_method_
                << "\n trunc_dist: " << trunc_dist_.transpose()
                << "\n save_immediate_result: " << save_immediate_result_
                << "\n evaluate_mme_: " << evaluate_mme_
                << "\n evaluate_gt_mme_: " << evaluate_gt_mme_
                << "\n mme search radius: " << nn_radius_
                << "\n evaluate_using_initial_: " << evaluate_using_initial_
                << "\n evaluate_noised_gt_: " << evaluate_noised_gt_
                << "\n noise_std_dev_: " << noise_std_dev_
                << "\n vmd_voxel_size_: " << vmd_voxel_size_
                << "\n "
                   "===========================================================\n ";
    }
};


class MapEval {
public:
    MapEval(Param &param)
            : param_(param), map_3d_(new PointCloud), gt_3d_(new PointCloud), processed_points(9) {
        param_.printParam();

        t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0.0;
        map_3d_render_inlier.reset(new PointCloud());
        map_3d_render_raw.reset(new PointCloud());
        map_3d_entropy.reset(new PointCloud());
        gt_3d_entropy.reset(new PointCloud());
        corresponding_cloud_est.reset(new PointCloud());
        corresponding_cloud_gt.reset(new PointCloud());

        gt_mesh.reset(new Mesh());
        est_mesh.reset(new Mesh());
        gt_mesh_filtered.reset(new Mesh());
        est_mesh_filtered.reset(new Mesh());

        // 根据pcd_file_name确定子文件夹名称
        if (param_.pcd_file_name_ == "merged_maps_all_trans.pcd") {
            subfolder = "merged_maps_all_results/";
        } else if (param_.pcd_file_name_ == "merged_maps_s0_trans.pcd") {
            subfolder = "merged_maps_s0_results/";
        } else if (param_.pcd_file_name_ == "merged_maps_s1_trans.pcd") {
            subfolder = "merged_maps_s1_results/";
        } else {
            subfolder = "map_results/";
            std::cerr << "Important: Invalid PCD file name: " << param_.pcd_file_name_ << std::endl;
        }

        // 创建子文件夹(如果不存在)
        results_subfolder = param_.evaluation_map_pcd_path_ + subfolder;
        std::cout << "Set Save results in " << results_subfolder << std::endl;
        if (!fs::exists(results_subfolder)) {
            fs::create_directory(results_subfolder);
        }
        // 将评估结果保存到对应的子文件夹中
        results_file_path = results_subfolder + "map_results.txt";
        file_result.open(results_file_path, std::ios::app);

        // Get current time
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        // Format the time as a string (you can adjust the format as needed)
        std::stringstream time_stream;
        time_stream << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X"); // Example format: YYYY-MM-DD HH:MM:SS
        // Output name and current time to the file
        file_result << param_.name_ << "======================= " << time_stream.str() << "======================= "
                    << std::endl;
        file_result << param_.map_gt_path_ << std::endl;
        file_result << param_.evaluation_map_pcd_path_ + param_.pcd_file_name_ << std::endl;

    }

    ~MapEval() {
        file_result.close();
    }

    int process();

    void computeMME();

    void computeMME(shared_ptr<PointCloud> &cloud_, shared_ptr<PointCloud> &cloud_gt_);

    void performRegistration();

    void calculateVMD();

    void saveMmeResults();

    void saveRegistrationResults();

    template<typename T>
    map<string, double> calculateError(vector<T> &result_vec);

    vector<double> computePointCloudDistance(
            shared_ptr<PointCloud> &reference_points,
            shared_ptr<PointCloud> &target_points);

    std::shared_ptr<PointCloud> renderDistanceOnPointCloud(
            std::shared_ptr<PointCloud> &reference_points,
            std::shared_ptr<PointCloud> &target_points, const double &dis);

    std::shared_ptr<open3d::geometry::PointCloud> ColorPointCloudByMME(
            const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
            const std::vector<double> &entropies,
            double mean_entropy);

    std::shared_ptr<open3d::geometry::PointCloud> ColorPointCloudByMME(
            const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
            const std::vector<double> &entropies,
            double min_abs_entropy, double max_abs_entropy);

    std::shared_ptr<open3d::geometry::PointCloud> ColorPointCloudByMME(
            const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
            const std::vector<double> &entropies);


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
                          geometry::PointCloud &source_set,
                          geometry::PointCloud &target_set);

    void getDiffRegResult(std::vector<Vector5d> &result,
                          pipelines::registration::CorrespondenceSet &points_set,
                          geometry::PointCloud &source,
                          geometry::PointCloud &target,
                          Eigen::MatrixXd &source_set,
                          Eigen::MatrixXd &target_set);

    void getDiffRegResultWithCorrespondence(
            std::vector<Vector5d> &result,
            pipelines::registration::CorrespondenceSet &points_set,
            geometry::PointCloud &source,
            geometry::PointCloud &target,
            geometry::PointCloud &source_set,
            geometry::PointCloud &target_set);

    void calculateMetrics(pipelines::registration::RegistrationResult &registration_result);

    void calculateMetricsWithInitialMatrix();

    void saveResults();

    pipelines::registration::RegistrationResult performICPRegistration();

    double computeChamferDistance(const geometry::PointCloud &cloud1,
                                  const geometry::PointCloud &cloud2);


    double ComputeEntropy(const Eigen::Matrix3d &covariance);

    double ComputeMeanMapEntropy(const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
                                 std::vector<double> &entropies, double radius);

    double ComputeMeanMapEntropyUsingNormal(const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
                                            std::vector<double> &entropies, double radius);

    void StartProcessing(int total);  // 开始处理的函数声明

    void addGaussianNoise(std::shared_ptr<open3d::geometry::PointCloud> &cloud,
                          double noise_std_dev);

    // 非均匀密度变化
    void addNonUniformDensity(std::shared_ptr<open3d::geometry::PointCloud> &cloud,
                              double sparse_ratio,    // 稀疏区域保留比例
                              double dense_ratio,     // 密集区域保留比例
                              double region_size);    // 区域大小
    // 添加离群点
    void addSparseOutliers(std::shared_ptr<open3d::geometry::PointCloud> &cloud,
                           double outlier_ratio,      // 离群点比例
                           double outlier_range);     // 离群点最大距离
    // 局部几何变形
    void addLocalDeformation(std::shared_ptr<open3d::geometry::PointCloud> &cloud,
                             double deform_radius,     // 变形区域半径
                             double deform_strength,   // 变形强度
                             Eigen::Vector3d center);  // 变形中心

private:
    // 辅助函数：计算点到中心的距离
    double computeDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

public:
    Param param_;

private:
    shared_ptr<PointCloud> map_3d_, gt_3d_, noised_gt_3d_;
    double t1, t2, t3, t4, t5, t6, t7, t_fcd, t_acc;
    double t_vmd, t_v, t_cdf, t_scs;
    shared_ptr<PointCloud> map_3d_render_inlier, map_3d_render_raw;
    shared_ptr<PointCloud> map_3d_entropy, gt_3d_entropy;
    shared_ptr<PointCloud> corresponding_cloud_est, corresponding_cloud_gt;
    std::vector<Vector5d> est_gt_results, gt_est_results;
    Vector5d f1_vec = Vector5d::Zero();
    Vector5d cd_vec = Vector5d::Zero();
    Vector5d iou_vec = Vector5d::Zero();
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    double vmd = 0.0;

    double full_chamfer_dist = 0.0;

    double scs_overall = 0.0;
    bool eva_mesh = false;
    shared_ptr<Mesh> gt_mesh, est_mesh;
    shared_ptr<Mesh> gt_mesh_filtered, est_mesh_filtered;

    std::string subfolder;
    std::string results_subfolder;
    std::ofstream file_result;
    std::string results_file_path;

    std::mutex file_mt;

    std::vector<double> est_entropies, gt_entropies;
    std::vector<bool> valid_entropy_points;
    double mme_est = 0.0, mme_gt = 0.0;
    double max_abs_entropy = 0.0, min_abs_entropy = 0.0;
    double mean_entropy = 0.0;
    int valid_points = 0;
    std::atomic<int> processed_points;
    std::chrono::steady_clock::time_point start_time;
    int total_points = 0;

    double lambda_ = 1.0;  // Lamé 第一参数
    double mu_ = 0.5;      // Lamé 第二参数
    double alpha_ = 0.1;     // 平移-形变权重
};

#endif// SRC_POSE_SLAM_PRIOR_SRC_BENCHMARK_CLOUD_MAP_EVAL_H_
