#include "map_eval.h"

int MapEval::process() {
    TicToc tic_toc;
    io::ReadPointCloudOption io_params("auto", true, true, true);

    // load ground truth point cloud
    if (param_.map_gt_path_.substr(param_.map_gt_path_.find_last_of(".") + 1) == "pcd") {
        io::ReadPointCloudFromPCD(param_.map_gt_path_, *gt_3d_, io_params);
    } else if (param_.map_gt_path_.substr(param_.map_gt_path_.find_last_of(".") + 1) == "ply") {
        io::ReadPointCloudFromPLY(param_.map_gt_path_, *gt_3d_, io_params);
    } else { std::cerr << "Unsupported ground truth file format: " << param_.map_gt_path_ << std::endl; }


    Eigen::Vector3d center;
    if (param_.evaluate_noised_gt_) {
        // Experimental mode: add noise to gt cloud
        std::cout << "add noise to gt cloud!" << std::endl;
        // add noise to gt cloud for evaluation
        // 创建带噪声的GT点云副本
        auto noised_gt_3d_raw = std::make_shared<open3d::geometry::PointCloud>(*gt_3d_);
        // 根据实验类型选择不同的变形方式
        int noise_type_ = 1;
        switch (noise_type_) {
            case 0:
                addNonUniformDensity(noised_gt_3d_raw, 0.1, 0.9, 2.0); // 10%和90%的保留率
                break;
            case 1:
                //  add outlier
                addSparseOutliers(noised_gt_3d_raw, 0.1, 1); // 1%的离群点，最大1米偏移
                break;
            case 2:
                // 在点云中心附近创建变形
                center = noised_gt_3d_raw->GetCenter();
                addLocalDeformation(noised_gt_3d_raw, 5.0, 0.5, center); // 2米半径，0.5米强度
                break;
            case 3:
                addGaussianNoise(noised_gt_3d_raw, param_.noise_std_dev_);
                break;
        }
        map_3d_ = noised_gt_3d_raw;
        param_.initial_matrix_.setIdentity();
    } else {
        // normal mode: load the point cloud to be evaluated
        bool flag = io::ReadPointCloudFromPCD(param_.evaluation_map_pcd_path_ + param_.pcd_file_name_,
                                              *map_3d_, io_params);
        std::cout << "Estimate map pcd : " << param_.evaluation_map_pcd_path_ + param_.pcd_file_name_
                  << std::endl;
        if (!flag) return -1;
    }

    if (map_3d_->IsEmpty() || gt_3d_->IsEmpty()) {
        cout << "Empty point cloud detected" << endl;
        return -1;
    }
    // Downsampling for efficiency
    map_3d_ = map_3d_->VoxelDownSample(0.01);
    gt_3d_ = gt_3d_->VoxelDownSample(0.01);
    //    open3d::io::WritePointCloud(resul bvts_subfolder + "gt_map_1cm.pcd",
    //                                *gt_3d_);

    file_result << std::fixed << std::setprecision(15) << "est-gt points number: "
                << map_3d_->points_.size() << " " << gt_3d_->points_.size() << std::endl;
    std::cout << "1. load point size: " << map_3d_->points_.size() << ", "   << gt_3d_->points_.size() << ", " << t1 / 1000.0 << " [s]"  << std::endl;
    t1 = tic_toc.toc();

    //    map_3d_->EstimateNormals(geometry::KDTreeSearchParamHybrid(0.2, 20));
    if (param_.evaluate_mme_) {
        std::cout << "3. MM calculate....." << std::endl;
        // 创建 map_3d 点云的副本
        // std::shared_ptr<PointCloud> map_3d_copy = std::make_shared<PointCloud>(*map_3d_);
        // std::shared_ptr<PointCloud> gt_3d_copy = std::make_shared<PointCloud>(*map_3d_);
        computeMME(map_3d_, gt_3d_);
        std::cout << "3.1. MME calculation finished! Start to save results." << std::endl;
        if (param_.save_immediate_result_)
            saveMmeResults();
    }
    t2 = tic_toc.toc();
    std::cout << "3.2. estimate MME time: " << (t2 - t1) / 1000.0 << " [s]" << std::endl;

    std::cout << "4. Registration start! " << std::endl;
    // 点云匹配继续使用原始的 map_3d 点云
    // calculate AC/CD/COM
    if (param_.evaluate_using_initial_) {
        // 直接使用初始矩阵计算map metric
        std::cout << "4.1 Using initial matrix! " << std::endl;
        calculateMetricsWithInitialMatrix();
    } else {
        std::cout << "4.2 Using ICP! " << std::endl;
        performRegistration();
    }
    std::cout << "4. Registration calculation finished! Start to save results." << std::endl;

    // calculate AWD and SCS
    calculateVMD();
    std::cout << "4.5 VMD calculation finished!" << std::endl;

    std::cout << "5.0 Save results begin!" << std::endl;
    if (param_.save_immediate_result_)
        saveRegistrationResults();

    std::cout << "5. Save Results Finished! " << std::endl;
    return 0;
}


void MapEval::computeMME() {
    if (param_.evaluate_mme_) {
        TicToc tic_toc;
        // double radius = 0.2;
        // mme_est = ComputeMeanMapEntropy(map_3d_, est_entropies, param_.nn_radius_);
        mme_est = ComputeMeanMapEntropyUsingNormal(map_3d_, est_entropies, param_.nn_radius_);

        // 合并两个点云的熵值向量
        // merge the entropy vectors of the two point clouds for proper scaling to colored the entropy cloud
        //        std::vector<double> combined_entropies = est_entropies;  // 假设entropies已经包含了map_3d_的熵值
        //        combined_entropies.insert(combined_entropies.end(), gt_entropies.begin(), gt_entropies.end());  // 添加gt_3d_的熵值
        //        std::vector<double> non_zero_combined;
        //        std::copy_if(combined_entropies.begin(), combined_entropies.end(), std::back_inserter(non_zero_combined),
        //                     [](double val) { return val != 0.0; });
        //        max_abs_entropy = std::fabs(*std::min_element(non_zero_combined.begin(), non_zero_combined.end()));
        //        min_abs_entropy = std::fabs(*std::max_element(non_zero_combined.begin(), non_zero_combined.end()));
        //        std::cout << "MAX MIN ENTROPY: " << max_abs_entropy << " " << min_abs_entropy << std::endl;

        if (param_.evaluate_gt_mme_) {
            std::cout << "Caculate GT MME... "<< std::endl;
            mme_gt = ComputeMeanMapEntropy(gt_3d_, gt_entropies, param_.nn_radius_);
            std::cout << "MME EST-GT: " << mme_est << " " << mme_gt << std::endl;
        } else {
            std::cout << "MME EST: " << mme_est << std::endl;
        }
        map_3d_entropy = ColorPointCloudByMME(map_3d_, est_entropies);
        if (param_.evaluate_gt_mme_) {
            /** since we finally scale the range of entropy, no need to use the common max and min value*/
            gt_3d_entropy = ColorPointCloudByMME(gt_3d_, gt_entropies);
        }
        std::cout << "3. MME Cost Time: " << (tic_toc.toc() - t2) / 1000.0 << " [s]" << std::endl;
    }

}

void MapEval::computeMME(shared_ptr<PointCloud> &cloud_, shared_ptr<PointCloud> &cloud_gt_) {
    if (param_.evaluate_mme_) {
        TicToc tic_toc;
        // double radius = 0.1;
        //mme_est = ComputeMeanMapEntropy(map_3d_, est_entropies, radius);
        mme_est = ComputeMeanMapEntropyUsingNormal(cloud_, est_entropies, param_.nn_radius_);

        // 合并两个点云的熵值向量
        //        std::vector<double> combined_entropies = est_entropies;  // 假设entropies已经包含了map_3d_的熵值
        //        combined_entropies.insert(combined_entropies.end(), gt_entropies.begin(), gt_entropies.end());  // 添加gt_3d_的熵值
        //        std::vector<double> non_zero_combined;
        //        std::copy_if(combined_entropies.begin(), combined_entropies.end(), std::back_inserter(non_zero_combined),
        //                     [](double val) { return val != 0.0; });
        //        max_abs_entropy = std::fabs(*std::min_element(non_zero_combined.begin(), non_zero_combined.end()));
        //        min_abs_entropy = std::fabs(*std::max_element(non_zero_combined.begin(), non_zero_combined.end()));
        //        std::cout << "MAX MIN ENTROPY: " << max_abs_entropy << " " << min_abs_entropy << std::endl;
        if (param_.evaluate_gt_mme_) {
            mme_gt = ComputeMeanMapEntropy(cloud_gt_, gt_entropies, param_.nn_radius_);
            std::cout << "MME EST-GT: " << mme_est << " " << mme_gt << std::endl;
        } else {
            std::cout << "MME EST: " << mme_est << std::endl;
        }
        map_3d_entropy = ColorPointCloudByMME(cloud_, est_entropies);
        if (param_.evaluate_gt_mme_) {
            /** since we finally scale the range of entropy, no need to use the common max and min value*/
            gt_3d_entropy = ColorPointCloudByMME(cloud_gt_, gt_entropies);
        }
        std::cout << "2. Calculate MME: " << (tic_toc.toc() - t2) / 1000.0 << " [s]" << std::endl;
    }
}

void MapEval::performRegistration() {
    TicToc tic_toc;
    /** if building mesh */
    if (eva_mesh) {
        gt_mesh = createMeshFromPCD(gt_3d_, 0.6, 10);
        //    gt_mesh_filtered = gt_mesh->FilterSmoothLaplacian(10, 0.5);
        //    gt_mesh_filtered->ComputeVertexNormals();
        //    gt_mesh_filtered->PaintUniformColor({1, 0.7, 0});
        //    visualization::DrawGeometries({gt_mesh_filtered}, "mesh result");
        est_mesh = createMeshFromPCD(map_3d_, 0.6, 10);
        t3 = tic_toc.toc();
        std::cout << "3. create mesh: " << (t3) / 1000.0 << " [s]" << std::endl;
    }

    // ICP registration
    auto registration_result = performICPRegistration();
    t4 = tic_toc.toc();

    std::cout << "4. aligned cloud : " << (t4 - t3) / 1000.0 << " [s]"
              << std::endl;
    std::cout << "aligned transformation: \n" << trans << std::endl;
    std::cout << "icp overlap ratio: " << registration_result.fitness_
              << std::endl;
    std::cout << "icp correspondences rmse: " << registration_result.inlier_rmse_
              << std::endl;
    std::cout << "icp correspondences size: "
              << registration_result.correspondence_set_.size() << std::endl;

    file_result << std::fixed << setprecision(5) << "Aligned cloud: " << trans.matrix() << std::endl;
    file_result << std::fixed << setprecision(5) << "Aligned results: " << registration_result.fitness_ << " "
                << registration_result.correspondence_set_.size() << std::endl;
    std::cout << "Aligned Results saved to " << results_file_path << std::endl;


    // Calculate metrics and evaluate
    //t5 = tic_toc.toc();
    calculateMetrics(registration_result);
    t5 = tic_toc.toc();
    std::cout << "6. calculate est-gt and gt-est metrics : " << (t7 - t5) / 1000.0 << " [s]"
              << std::endl;
}


void MapEval::calculateVMD() {

    TicToc ticToc;

    // voxel
    VoxelCalculator voxelCalculator;
    // Create VoxelCalculator instances
    // double voxel_size = 3.0;  // Adjust this value as needed
    VoxelCalculator gt_calculator(param_.vmd_voxel_size_);
    VoxelCalculator est_calculator(param_.vmd_voxel_size_);
    // Build GT voxel map
    gt_calculator.buildVoxelMap(*gt_3d_);
    est_calculator.buildVoxelMap(*map_3d_);
    // Update EST voxel map to mark active, new, and old areas
    est_calculator.updateVoxelMap(gt_calculator.getVoxelMap());
    t_v = ticToc.toc();

    // Compute Wasserstein distances and save results
    std::ofstream output_file(results_subfolder + "voxel_errors.txt");
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output file." << std::endl;
        //return 1;
    }
    const VoxelMap &gt_map = gt_calculator.getVoxelMap();
    const VoxelMap &est_map = est_calculator.getVoxelMap();
    std::unordered_map<Eigen::Vector3i, double, hash_eigen<Eigen::Vector3i>> wasserstein_distances;
    for (const auto &kv : est_map) {
        const Eigen::Vector3i &index = kv.first;
        const VoxelInfo &est_voxel = kv.second;
        // Only process active voxels
        if (est_voxel.active == 1) {
            auto gt_it = gt_map.find(index);
            if (gt_it != gt_map.end()) {
                const VoxelInfo &gt_voxel = gt_it->second;
                if (est_voxel.num_points < 100 || gt_voxel.num_points < 100)
                    continue;
                double ws_distance = est_calculator.computeWassersteinDistanceGaussian(gt_voxel, est_voxel);
                wasserstein_distances[index] = ws_distance;
                // Calculate voxel boundaries
                Eigen::Vector3d voxel_min = index.cast<double>() * param_.vmd_voxel_size_;
                Eigen::Vector3d voxel_max = (index.cast<double>() + Eigen::Vector3d::Ones()) * param_.vmd_voxel_size_;
                output_file << voxel_min.x() << " " << voxel_min.y() << " " << voxel_min.z() << " "
                            << voxel_max.x() << " " << voxel_max.y() << " " << voxel_max.z() << " "
                            << est_voxel.mu.x() << " " << est_voxel.mu.y() << " " << est_voxel.mu.z() << " "
                            << ws_distance << " " << gt_voxel.num_points << " " << est_voxel.num_points << " "
                            << est_voxel.sigma(0, 0) << " " << est_voxel.sigma(0, 1) << " " << est_voxel.sigma(0, 2)
                            << " " << est_voxel.sigma(1, 1) << " " << est_voxel.sigma(1, 2) << " "
                            << est_voxel.sigma(2, 2) << " "
                            << gt_voxel.mu.x() << " " << gt_voxel.mu.y() << " " << gt_voxel.mu.z() << " "
                            << gt_voxel.sigma(0, 0) << " " << gt_voxel.sigma(0, 1) << " " << gt_voxel.sigma(0, 2)
                            << " " << gt_voxel.sigma(1, 1) << " " << gt_voxel.sigma(1, 2) << " "
                            << gt_voxel.sigma(2, 2)
                            << std::endl;
            }
        }
    }
    output_file.close();
    std::cout << "voxel errors Results saved to " << results_subfolder + "voxel_errors.txt" << std::endl;

    std::vector<double> ws_distances;
    for (const auto &kv : wasserstein_distances) {
        ws_distances.push_back(kv.second);
    }
    double mean_ws = std::accumulate(ws_distances.begin(), ws_distances.end(), 0.0) / ws_distances.size();
    vmd = mean_ws;
    t_vmd = ticToc.toc();
    std::cout << "Calculate VMD: " << mean_ws << std::endl;


    // std::cout << "Calculate CDF!" << std::endl;
    double variance_ws = 0.0;
    for (double w : ws_distances) {
        variance_ws += (w - mean_ws) * (w - mean_ws);
    }
    variance_ws /= ws_distances.size();
    // exists bugs here, wrong codes according to the eq.9
    // double std_dev_ws = std::sqrt(variance_ws);
    // double w_3sigma = mean_ws + 3 * std_dev_ws;
    // std::cout << "3-Sigma Threshold: " << w_3sigma << std::endl;

    std::sort(ws_distances.begin(), ws_distances.end());
    std::ofstream cdf_file(results_subfolder + "voxel_wasserstein_cdf.txt");
    if (!cdf_file.is_open()) {
        std::cerr << "Failed to open CDF output file." << std::endl;
    }
    for (size_t i = 0; i < ws_distances.size(); ++i) {
        double cdf_value = static_cast<double>(i + 1) / ws_distances.size();
        cdf_file << ws_distances[i] << " " << cdf_value << std::endl;
    }
    cdf_file.close();
    t_cdf = ticToc.toc();
    std::cout << "CDF results saved to " << results_subfolder + "voxel_wasserstein_cdf.txt" << std::endl;

    // calculate SCS score
    std::cout << "Calculate SScore..." << std::endl;
    double total_scs = 0.0;
    int scs_count = 0;
    int radius = 5; // Adjust the radius as needed
    for (const auto &kv : wasserstein_distances) {
        const Eigen::Vector3i &index = kv.first;
        double ws_distance = kv.second;
        // Get neighboring voxel indices
        std::vector<Eigen::Vector3i> neighbors = est_calculator.getNeighborIndices(index, radius);
        std::vector<double> neighbor_ws_distances;
        for (const auto &neighbor_index : neighbors) {
            auto it = wasserstein_distances.find(neighbor_index);
            if (it != wasserstein_distances.end()) {
                neighbor_ws_distances.push_back(it->second);
            }
        }
        if (!neighbor_ws_distances.empty()) {
            double mean_ws = std::accumulate(neighbor_ws_distances.begin(), neighbor_ws_distances.end(), 0.0) /
                             neighbor_ws_distances.size();
            double variance_ws = 0.0;
            for (double w : neighbor_ws_distances) {
                variance_ws += (w - mean_ws) * (w - mean_ws);
            }
            variance_ws /= neighbor_ws_distances.size();
            double std_dev_ws = std::sqrt(variance_ws);
            double scs = std_dev_ws / mean_ws;
            total_scs += scs;
            scs_count++;
        }
    }
    scs_overall = total_scs / scs_count;
    t_scs = ticToc.toc();
    std::cout << "Spatial Consistency Score (SCS): " << scs_overall << std::endl;
}

void MapEval::saveMmeResults() {
    if (param_.evaluate_mme_) {
        file_mt.lock();
        file_result << std::fixed << setprecision(5) << "MME: " << mme_est << " " << mme_gt << " "
                    << min_abs_entropy << " " << max_abs_entropy << std::endl;
        file_mt.unlock();
        std::cout << "MME Results Saved to " << results_file_path << std::endl;

        // save cloud
        open3d::io::WritePointCloud(results_subfolder + "map_entropy.pcd", *map_3d_entropy);
        std::cout << "Save rendered entropy map to "
                  << results_subfolder + "map_entropy.pcd" << std::endl;
        if (param_.evaluate_gt_mme_) {
            open3d::io::WritePointCloud(results_subfolder + "gt_entropy.pcd", *gt_3d_entropy);
            std::cout << "Save rendered entropy gt map to " << results_subfolder + "gt_entropy.pcd" << std::endl;
        }
    }
}

void MapEval::saveRegistrationResults() {

    std::cout << "AC+MME Time: " << t_acc + (t2 - t1) / 1000.0 << std::endl;
    std::cout << "CD+MME Time: " << t_fcd + (t2 - t1) / 1000.0 << std::endl;
    std::cout << "AWD+SCS Time: " << t_v / 1000.0 + (t_vmd - t_v) / 1000.0 + (t_scs - t_cdf) / 1000.0 << std::endl;

    // save file results
    file_mt.lock();
    file_result << std::fixed << std::setprecision(15) << "RMSE: "
                << est_gt_results.at(1).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Mean: "
                << est_gt_results.at(0).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Std: "
                << est_gt_results.at(3).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Comp: "
                << est_gt_results.at(2).transpose() << std::endl;
    // file_result << std::fixed << setprecision(5) << "CD: " << cd_vec.transpose()
    //             << std::endl;
    file_result << std::fixed << setprecision(5) << "FULL CD: " << full_chamfer_dist << std::endl;
    // file_result << std::fixed << setprecision(5) << "F1: " << f1_vec.transpose()
    //             << std::endl;
    // file_result << std::fixed << setprecision(5) << "IOU: " << iou_vec.transpose()
    //             << std::endl;
    file_result << std::fixed << setprecision(5) << "VMD: " << vmd << std::endl;
    file_result << std::fixed << setprecision(5) << "SCS: " << scs_overall << std::endl;
    file_result << "Time load-MME-mesh-ICP-Metric-AC-FCD: " << t1 / 1000.0 << " " << (t2 - t1) / 1000.0 << " "
                << (t3) / 1000.0 << " " << (t4 - t3) / 1000.0 << " "
                << (t5 - t4) / 1000.0 << " " << t_acc << " " << t_fcd << std::endl;
    file_result << "VMD Time voxelization-WD-CDF-SCS: " << t_v / 1000.0 << " " << (t_vmd - t_v) / 1000.0 << " "
                << (t_cdf - t_vmd) / 1000.0 << " " << (t_scs - t_cdf) / 1000.0 << std::endl;
    file_result << "AC+MME Time: " << t_acc + (t2 - t1) / 1000.0 << std::endl;
    file_result << "CD+MME Time: " << t_fcd + (t2 - t1) / 1000.0 << std::endl;
    file_result << "AWD+SCS Time: " << t_v / 1000.0 + (t_vmd - t_v) / 1000.0 + (t_scs - t_cdf) / 1000.0 << std::endl;

    file_result.close();
    file_mt.unlock();
    std::cout << "5.1 Save txt files finished " << results_subfolder + "map_results.txt" << std::endl;

    map_3d_render_inlier = renderDistanceOnPointCloud(
            corresponding_cloud_gt, corresponding_cloud_est, param_.trunc_dist_[0]);
    map_3d_render_raw =
            renderDistanceOnPointCloud(gt_3d_, map_3d_, param_.trunc_dist_[0]);
    open3d::io::WritePointCloud(results_subfolder + "raw_rendered_dis_map.pcd",
                                *map_3d_render_raw);
    std::cout << "5.2 Save rendered inlier distance error map to " << results_subfolder + "raw_rendered_dis_map.pcd"
              << std::endl;
    open3d::io::WritePointCloud(results_subfolder + "inlier_rendered_dis_map.pcd", *map_3d_render_inlier);
    std::cout << "5.3 Save rendered raw distance error map to " << results_subfolder + "inlier_rendered_dis_map.pcd"
              << std::endl;

    if (param_.evaluate_noised_gt_) {
        open3d::io::WritePointCloud(results_subfolder + "noise_gt_map.pcd", *map_3d_);
        std::cout << "5.4 Save noise_gt_map to " << results_subfolder + "noise_gt_map.pcd"
                  << std::endl;
    }

    // ToDO: can not save mesh file successfully
    if (eva_mesh) {
        // create corresspondence mesh
        shared_ptr<Mesh> correspdence_mesh(new Mesh());
        if (1) {
            corresponding_cloud_est->EstimateNormals(
                    geometry::KDTreeSearchParamHybrid(1.0, 30));
            //correspdence_mesh = createMeshFromPCD(corresponding_cloud_est, 0.6, 11);
        }
        // visualization::DrawGeometries({correspdence_mesh}, "correspdence_mesh");

        // TODO : render mesh by distance error
        std::cout << "render correspondence mesh begin!" << std::endl;
        // gt mesh only save once
        open3d::io::WriteTriangleMesh(results_subfolder + "gt_mesh.ply", *gt_mesh);
        open3d::io::WriteTriangleMesh(results_subfolder + "est_mesh.ply", *est_mesh);
        open3d::io::WriteTriangleMesh(results_subfolder + "correspondence_mesh.ply", *correspdence_mesh);
    }
    visualization::DrawGeometries({map_3d_render_raw}, "mesh result");
}


template<typename T>
std::map<std::string, double> MapEval::calculateError(
        std::vector<T> &result_vec) {
    double mean = 0.0;
    double rmse = 0.0;
    for (auto value : result_vec) {
        mean += value;
        rmse += std::pow(value, 2.0);
    }
    if (!result_vec.empty()) {
        mean /= static_cast<double>(result_vec.size());
        rmse = std::sqrt(rmse / static_cast<double>(result_vec.size()));
    }
    double stddev = 0.0;
    for (auto value : result_vec) {
        stddev += std::pow(value - mean, 2.0);
    }
    if (result_vec.size() > 2) {
        stddev = sqrt(stddev / static_cast<double>(result_vec.size() - 1));
    }
    std::map<std::string, double> result_error;
    result_error.emplace(std::string("mean"), mean);
    result_error.emplace(std::string("rmse"), rmse);
    result_error.emplace(std::string("stddev"), stddev);
    return result_error;
}

std::vector<double> MapEval::computePointCloudDistance(
        std::shared_ptr<PointCloud> &reference_points,
        std::shared_ptr<PointCloud> &target_points) {
    open3d::geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*reference_points);
    int num_neighbors = 1;
    std::vector<double> eval_results;
    eval_results.reserve(target_points->points_.size());
    for (size_t i = 0; i < target_points->points_.size(); ++i) {
        std::vector<int> new_indices_vec;
        std::vector<double> new_dists_vec;
        kdtree.SearchKNN(target_points->points_[i], num_neighbors, new_indices_vec,
                         new_dists_vec);
        eval_results.push_back(new_dists_vec.front());
    }
    return eval_results;
}

std::shared_ptr<PointCloud> MapEval::renderDistanceOnPointCloud(
        std::shared_ptr<PointCloud> &reference_points,
        std::shared_ptr<PointCloud> &target_points, const double &dis) {
    std::vector<double> eval_dis =
            computePointCloudDistance(reference_points, target_points);
    for (size_t i = 0; i < eval_dis.size(); i++) {
        if (eval_dis[i] > dis) {
            eval_dis[i] = dis;
        }
    }

    std::shared_ptr<PointCloud> target_points_render(
            new PointCloud(*target_points));
    target_points_render->PaintUniformColor(Eigen::Vector3d(1, 1, 1));
    open3d::visualization::ColorMapJet colorbar;
    for (int i = 0; i < eval_dis.size(); i++) {
        double a = eval_dis[i] / dis;
        target_points_render->colors_[i] = colorbar.GetColor(a);
    }
    return target_points_render;
}


// 新增：根据MME上色
std::shared_ptr<open3d::geometry::PointCloud> MapEval::ColorPointCloudByMME(
        const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
        const std::vector<double> &entropies,
        double mean_entropy) {
    // 创建一个新的点云用于上色
    auto colored_pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    colored_pointcloud->points_ = pointcloud->points_;  // 只复制点的位置信息
    colored_pointcloud->colors_.resize(colored_pointcloud->points_.size(), Eigen::Vector3d(1, 1, 1));
    //colored_pointcloud->PaintUniformColor(Eigen::Vector3d(1, 1, 1));

    // 计算熵值的绝对值范围
    std::vector<double> non_zero_entropies;
    std::copy_if(entropies.begin(), entropies.end(), std::back_inserter(non_zero_entropies),
                 [](double entropy) { return entropy != 0.0; });
    double max_abs_entropy = std::fabs(*std::min_element(non_zero_entropies.begin(), non_zero_entropies.end()));
    double min_abs_entropy = std::fabs(*std::max_element(non_zero_entropies.begin(), non_zero_entropies.end()));

    std::cout << "MAX MIN ENTROPY: " << max_abs_entropy << " " << min_abs_entropy << std::endl;

    // 创建颜色映射
    open3d::visualization::ColorMapJet color_map;
    // 根据熵值对点云进行上色
    for (size_t i = 0; i < entropies.size(); ++i) {
        if (valid_entropy_points[i]) {
            // 将熵值的绝对值归一化到0到1的范围
            double normalized_entropy =
                    (std::fabs(entropies[i]) - min_abs_entropy) / (max_abs_entropy - min_abs_entropy);
            // 从色图中获取颜色
            Eigen::Vector3d color = color_map.GetColor(normalized_entropy);
            colored_pointcloud->colors_[i] = color;
        } else {
            // colored_pointcloud->colors_[i] = Eigen::Vector3d(1.0, 0., 0.);  // red色表示无效点
        }
    }
    return colored_pointcloud;
}

std::shared_ptr<open3d::geometry::PointCloud> MapEval::ColorPointCloudByMME(
        const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
        const std::vector<double> &entropies,
        double min_abs_entropy, double max_abs_entropy) {
    // 创建一个新的点云用于上色
    auto colored_pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    //colored_pointcloud->points_ = pointcloud->points_;  // 只复制点的位置信息
    // colored_pointcloud->colors_.resize(colored_pointcloud->points_.size(), Eigen::Vector3d(1, 1, 1));
    //colored_pointcloud->PaintUniformColor(Eigen::Vector3d(1, 1, 1));

    // 计算熵值的绝对值范围
//    std::vector<double> non_zero_entropies;
//    std::copy_if(entropies.begin(), entropies.end(), std::back_inserter(non_zero_entropies),
//                 [](double entropy) { return entropy != 0.0; });
//    max_abs_entropy = std::fabs(*std::min_element(non_zero_entropies.begin(), non_zero_entropies.end()));
//    min_abs_entropy = std::fabs(*std::max_element(non_zero_entropies.begin(), non_zero_entropies.end()));

    // 创建颜色映射
    open3d::visualization::ColorMapJet color_map;
    // 根据熵值对点云进行上色
    for (size_t i = 0; i < entropies.size(); ++i) {
        if (valid_entropy_points[i]) {
            // 将熵值的绝对值归一化到0到1的范围
            double normalized_entropy =
                    (std::fabs(entropies[i]) - min_abs_entropy) / (max_abs_entropy - min_abs_entropy);
            // 从色图中获取颜色
            Eigen::Vector3d color = color_map.GetColor(normalized_entropy);
            //colored_pointcloud->colors_[i] = color;
            // 将有效的点和对应的颜色添加到新的点云
            colored_pointcloud->points_.push_back(pointcloud->points_[i]);
            colored_pointcloud->colors_.push_back(color);
        } else {
            //            colored_pointcloud->colors_[i] = Eigen::Vector3d(1.0, 0., 0.);  // red色表示无效点
        }
    }
    return colored_pointcloud;
}


std::shared_ptr<open3d::geometry::PointCloud> MapEval::ColorPointCloudByMME(
        const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
        const std::vector<double> &entropies) {

    // 创建一个新的点云用于上色
    auto colored_pointcloud = std::make_shared<open3d::geometry::PointCloud>();
//    colored_pointcloud->points_ = pointcloud->points_;  // 只复制点的位置信息
//    colored_pointcloud->colors_.resize(colored_pointcloud->points_.size(), Eigen::Vector3d(1, 1, 1));
//    colored_pointcloud->PaintUniformColor(Eigen::Vector3d(1, 1, 1));

    // 计算熵值的绝对值范围
    std::vector<double> non_zero_entropies;
    std::copy_if(entropies.begin(), entropies.end(), std::back_inserter(non_zero_entropies),
                 [](double entropy) { return entropy != 0.0; });
    max_abs_entropy = std::fabs(*std::min_element(non_zero_entropies.begin(), non_zero_entropies.end()));
    min_abs_entropy = std::fabs(*std::max_element(non_zero_entropies.begin(), non_zero_entropies.end()));

    // 创建颜色映射
    open3d::visualization::ColorMapJet color_map;
    // 根据熵值对点云进行上色
    for (size_t i = 0; i < entropies.size(); ++i) {
        if (valid_entropy_points[i]) {
            // 将熵值的绝对值归一化到0到1的范围
            // i want to normalize the  entropy value to 0-255,and represent the entropy value by jet color (Red->green->blue), blue stands for low entropy,
            //  red stands for high entropy which means lower map accuracy.
            // since entropy value is negative, i need to normalize the absolute value of entropy to 0-1, and then map it to 0-255
            // also entropy value may range from a small scale, for example, -5.5 to -6.5 , so i need to normalize the absolute value of entropy to 0-1
            // and then map it to 0-255
            double normalized_entropy =
                    (std::fabs(entropies[i]) - min_abs_entropy) / (max_abs_entropy - min_abs_entropy);
            //normalized_entropy = std::sqrt(normalized_entropy);

            double epsilon = 1e-1;
            double mapped_entropy = log(normalized_entropy + epsilon);
            // 将映射后的熵值再次归一化以适配颜色映射
            normalized_entropy = (mapped_entropy - log(epsilon)) / (log(1.0 + epsilon) - log(epsilon));

            // 从色图中获取颜色
            // 从色图中获取颜色
            Eigen::Vector3d color = color_map.GetColor(normalized_entropy);

            // 将有效的点和对应的颜色添加到新的点云
            colored_pointcloud->points_.push_back(pointcloud->points_[i]);
            colored_pointcloud->colors_.push_back(color);
        } else {
        //            colored_pointcloud->colors_[i] = Eigen::Vector3d(1.0, 0., 0.);  // red色表示无效点
        }
    }

    return colored_pointcloud;
}

std::shared_ptr<Mesh> MapEval::renderDistanceOnMesh(
        std::shared_ptr<PointCloud> &reference_points,
        std::shared_ptr<Mesh> &target_mesh,
        std::shared_ptr<PointCloud> &target_points, double dis) {
    vector<double> eval_dis =
            computePointCloudDistance(reference_points, target_points);

    std::shared_ptr<Mesh> target_mesh_render(new Mesh(*target_mesh));
    target_mesh_render->PaintUniformColor(Eigen::Vector3d(1, 0.7, 0));
    target_mesh_render->ComputeVertexNormals();
    visualization::ColorMapJet colorbar;

    std::cout << "reference target points size: "
              << reference_points->points_.size() << ", "
              << target_points->points_.size() << std::endl;

    std::cout << "rending mesh  vertices_ "
              << target_mesh_render->vertices_.size() << std::endl;
    std::cout << "rending mesh  vertex_colors_ "
              << target_mesh_render->vertex_colors_.size() << std::endl;

    std::cout << "rending mesh  vertex_normals_"
              << target_mesh_render->vertex_normals_.size() << std::endl;
    std::cout << "rending mesh  eval_dis " << eval_dis.size() << std::endl;
    // visualization::DrawGeometries({target_mesh_render}, "mesh result 1");

    for (size_t i = 0; i < eval_dis.size(); i++) {
        if (eval_dis[i] > dis) {
            eval_dis[i] = dis;
        }
        double a = eval_dis[i] / dis;
        target_mesh_render->vertex_colors_[i] = colorbar.GetColor(a);
    }
    visualization::DrawGeometries({target_mesh_render}, "mesh result 2");

    std::cout << "rending mesh  " << std::endl;
    return target_mesh_render;
}

shared_ptr<Mesh> MapEval::createMeshFromPCD(
        shared_ptr<PointCloud> &point_cloud, double density_thres, int depth) {
    auto mesh_tuple = Mesh::CreateFromPointCloudPoisson(*point_cloud, depth);

    shared_ptr<Mesh> mesh_ptr = std::get<0>(mesh_tuple);
    vector<double> densities_es = std::get<1>(mesh_tuple);
    vector<bool> densities_mask(densities_es.size(), false);
    mesh_ptr->PaintUniformColor({1, 1, 1});
    mesh_ptr->ComputeVertexNormals();

    int verties_size = mesh_ptr->vertices_.size();
    double min_den = 999999999999, max_den = 0.0, mean_den = 0.0;
    int dex = 0;
    for (int i = 0; i < densities_es.size(); ++i) {
        double density_tmp = densities_es.at(i);
        if (max_den <= density_tmp) max_den = density_tmp;
        if (min_den >= density_tmp) min_den = density_tmp;
        mean_den += density_tmp;
        //    if (density_tmp <= density_thres) {
        //      densities_mask.at(i) = true;
        //    } else
        //      dex++;
    }
    mean_den /= (double) densities_es.size();

    visualization::ColorMapJet colorbar;
    for (int i = 0; i < densities_es.size(); ++i) {
        double density_tmp = densities_es.at(i);
        double density_color = (density_tmp - min_den) / (max_den - min_den);
        mesh_ptr->vertex_colors_[i] = colorbar.GetColor(density_color);

        if (density_color < density_thres) {
            // filter
            densities_mask.at(i) = true;
        } else
            dex++;
    }

    mesh_ptr->RemoveVerticesByMask(densities_mask);
    std::cout << "reate mesh points size: " << point_cloud->points_.size()
              << std::endl;
    std::cout << "density: min mean max: " << min_den << " " << mean_den << " "
              << max_den << std::endl;
    std::cout << "vertices filter ratio: " << mesh_ptr->vertices_.size() << " / "
              << verties_size << " "
              << mesh_ptr->vertices_.size() / double(densities_es.size())
              << std::endl;
    open3d::visualization::DrawGeometries({mesh_ptr}, "mesh result");
    return mesh_ptr;
}

void MapEval::getDiffRegResult(
        std::vector<Vector5d> &result,
        pipelines::registration::CorrespondenceSet &points_set,
        geometry::PointCloud &source, geometry::PointCloud &target) {
    vector<double> est_gt_dis(points_set.size(), 0.0);

    Vector5i number_vec = Vector5i::Zero();
    Vector5d fitness_vec = Vector5d::Zero();
    Vector5d mean_vec = Vector5d::Zero();
    Vector5d rmse_vec = Vector5d::Zero();
    Vector5d sigma_vec = Vector5d::Zero();

    // accuracy (mean) and precision (standard deviation)
    for (int i = 0; i < points_set.size(); ++i) {
        Eigen::Vector2i points_i = points_set.at(i);
        auto map_pt = source.points_.at(points_i[0]);
        auto gt_pt = target.points_.at(points_i[1]);

        double norm_dis = (map_pt - gt_pt).norm();
        double squre_dis = (map_pt - gt_pt).squaredNorm();

        est_gt_dis.at(i) = norm_dis;
        // 20 cm
        if (norm_dis <= param_.trunc_dist_[0]) {
            mean_vec[0] += norm_dis;
            rmse_vec[0] += squre_dis;
            number_vec[0]++;
        }
        if (norm_dis <= param_.trunc_dist_[1]) {
            mean_vec[1] += norm_dis;
            rmse_vec[1] += squre_dis;
            number_vec[1]++;
        }
        if (norm_dis <= param_.trunc_dist_[2]) {
            mean_vec[2] += norm_dis;
            rmse_vec[2] += squre_dis;
            number_vec[2]++;
        }
        if (norm_dis <= param_.trunc_dist_[3]) {
            mean_vec[3] += norm_dis;
            rmse_vec[3] += squre_dis;
            number_vec[3]++;
        }
        if (norm_dis <= param_.trunc_dist_[4]) {
            mean_vec[4] += norm_dis;
            rmse_vec[4] += squre_dis;
            number_vec[4]++;
        }
    }

    mean_vec /= points_set.size();
    rmse_vec /= points_set.size();
    int source_num = source.points_.size();
    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
        fitness_vec[i] = number_vec[i] * 1.0 / source_num;
        rmse_vec[i] = sqrt(rmse_vec[i]);

        double sigma = 0.0;
        for (int j = 0; j < points_set.size(); ++j) {
            double error_dis = est_gt_dis.at(j) - mean_vec[i];
            sigma += std::pow(error_dis, 2);
        }
        sigma /= points_set.size();
        sigma_vec[i] = sqrt(sigma);
    }
    result.push_back(mean_vec);
    result.push_back(rmse_vec);
    result.push_back(fitness_vec);
    result.push_back(sigma_vec);
}

void MapEval::getDiffRegResult(
        std::vector<Vector5d> &result,
        pipelines::registration::CorrespondenceSet &points_set,
        geometry::PointCloud &source, geometry::PointCloud &target,
        Eigen::MatrixXd &source_set, Eigen::MatrixXd &target_set) {
    vector<double> est_gt_dis(points_set.size(), 0.0);

    Vector5i number_vec = Vector5i::Zero();
    Vector5d fitness_vec = Vector5d::Zero();
    Vector5d mean_vec = Vector5d::Zero();
    Vector5d rmse_vec = Vector5d::Zero();
    Vector5d sigma_vec = Vector5d::Zero();

    source_set.resize(3, points_set.size());
    target_set.resize(3, points_set.size());
    source_set.setZero();
    target_set.setZero();

    // accuracy (mean) and precision (standard deviation)
    int count = 0;
    for (int i = 0; i < points_set.size(); ++i) {
        Eigen::Vector2i points_i = points_set.at(i);
        auto map_pt = source.points_.at(points_i[0]);
        auto gt_pt = target.points_.at(points_i[1]);

        double norm_dis = (map_pt - gt_pt).norm();
        double squre_dis = (map_pt - gt_pt).squaredNorm();

        est_gt_dis.at(i) = norm_dis;
        // 20 cm
        if (norm_dis <= param_.trunc_dist_[0]) {
            mean_vec[0] += norm_dis;
            rmse_vec[0] += squre_dis;
            number_vec[0]++;
        }
        if (norm_dis <= param_.trunc_dist_[1]) {
            mean_vec[1] += norm_dis;
            rmse_vec[1] += squre_dis;
            number_vec[1]++;
        }
        if (norm_dis <= param_.trunc_dist_[2]) {
            mean_vec[2] += norm_dis;
            rmse_vec[2] += squre_dis;
            number_vec[2]++;
        }
        if (norm_dis <= param_.trunc_dist_[3]) {
            mean_vec[3] += norm_dis;
            rmse_vec[3] += squre_dis;
            number_vec[3]++;
        }
        if (norm_dis <= param_.trunc_dist_[4]) {
            mean_vec[4] += norm_dis;
            rmse_vec[4] += squre_dis;
            number_vec[4]++;
            source_set.block<3, 1>(0, count) = map_pt;
            target_set.block<3, 1>(0, count) = target_set;
            count++;
        }
    }
    // std::cout << source_set.matrix() << std::endl;
    source_set.conservativeResize(3, count);
    target_set.conservativeResize(3, count);

    //  MatrixXd source_final(3, count);
    //  MatrixXd target_final(3, count);

    //  source_final = source_set.topLeftCorner<3, count>(0,0);
    //  target_final = target_set.block<3, 1>(0, 0);

    mean_vec /= points_set.size();
    rmse_vec /= points_set.size();
    int source_num = source.points_.size();
    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
        fitness_vec[i] = number_vec[i] * 1.0 / source_num;
        rmse_vec[i] = sqrt(rmse_vec[i]);

        double sigma = 0.0;
        for (int j = 0; j < points_set.size(); ++j) {
            double error_dis = est_gt_dis.at(j) - mean_vec[i];
            sigma += std::pow(error_dis, 2);
        }
        sigma /= points_set.size();
        sigma_vec[i] = sqrt(sigma);
    }

    result.push_back(mean_vec);
    result.push_back(rmse_vec);
    result.push_back(fitness_vec);
    result.push_back(sigma_vec);
}

void MapEval::getDiffRegResult(std::vector<Vector5d> &result,
                                   pipelines::registration::CorrespondenceSet &points_set,
                                   geometry::PointCloud &source,
                                   geometry::PointCloud &target,
                                   geometry::PointCloud &source_set,
                                   geometry::PointCloud &target_set) {
    vector<double> est_gt_dis(points_set.size(), 0.0);

    Vector5d number_vec = Vector5d::Zero();
    Vector5d fitness_vec = Vector5d::Zero();
    Vector5d mean_vec = Vector5d::Zero();
    Vector5d rmse_vec = Vector5d::Zero();
    Vector5d sigma_vec = Vector5d::Zero();

    // accuracy (mean) and precision (standard deviation)
    for (int i = 0; i < points_set.size(); ++i) {
        Eigen::Vector2i points_i = points_set.at(i);
        auto map_pt = source.points_.at(points_i[0]);
        auto gt_pt = target.points_.at(points_i[1]);

        source_set.points_.push_back(map_pt);
        target_set.points_.push_back(gt_pt);

        double norm_dis = (map_pt - gt_pt).norm();
        double squre_dis = (map_pt - gt_pt).squaredNorm();
        est_gt_dis.at(i) = norm_dis;

        // 20 cm
        if (norm_dis <= param_.trunc_dist_[0]) {
            mean_vec[0] += norm_dis;
            rmse_vec[0] += squre_dis;
            number_vec[0]++;
        }
        if (norm_dis <= param_.trunc_dist_[1]) {
            mean_vec[1] += norm_dis;
            rmse_vec[1] += squre_dis;
            number_vec[1]++;
        }
        if (norm_dis <= param_.trunc_dist_[2]) {
            mean_vec[2] += norm_dis;
            rmse_vec[2] += squre_dis;
            number_vec[2]++;
        }
        if (norm_dis <= param_.trunc_dist_[3]) {
            mean_vec[3] += norm_dis;
            rmse_vec[3] += squre_dis;
            number_vec[3]++;
        }
        if (norm_dis <= param_.trunc_dist_[4]) {
            mean_vec[4] += norm_dis;
            rmse_vec[4] += squre_dis;
            number_vec[4]++;
        }
    }

    mean_vec /= points_set.size();
    rmse_vec /= points_set.size();
//    int source_num = source.points_.size();
    int target_num = source.points_.size();
    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
        fitness_vec[i] = number_vec[i] * 1.0 / target_num;
        rmse_vec[i] = sqrt(rmse_vec[i]);

        double sigma = 0.0;
        for (int j = 0; j < points_set.size(); ++j) {
            double error_dis = est_gt_dis.at(j) - mean_vec[i];
            sigma += std::pow(error_dis, 2);
        }
        sigma /= points_set.size();
        sigma_vec[i] = sqrt(sigma);
    }

    result.push_back(mean_vec);
    result.push_back(rmse_vec);
    result.push_back(fitness_vec);
    result.push_back(sigma_vec);
    result.push_back(number_vec);
}

void MapEval::getDiffRegResultWithCorrespondence(
        std::vector<Vector5d> &result,
        pipelines::registration::CorrespondenceSet &points_set,
        geometry::PointCloud &source,
        geometry::PointCloud &target,
        geometry::PointCloud &source_set,
        geometry::PointCloud &target_set) {
    vector<double> est_gt_dis(points_set.size(), 0.0);
    Vector5d number_vec = Vector5d::Zero();
    Vector5d fitness_vec = Vector5d::Zero();
    Vector5d mean_vec = Vector5d::Zero();
    Vector5d rmse_vec = Vector5d::Zero();
    Vector5d sigma_vec = Vector5d::Zero();
    // 清空source_set和target_set
    source_set.points_.clear();
    target_set.points_.clear();
    // 根据对应点信息更新source_set和target_set
    for (const auto &correspondence : points_set) {
        source_set.points_.push_back(source.points_[correspondence(0)]);
        target_set.points_.push_back(target.points_[correspondence(1)]);
    }
    // 计算精度和召回率
    for (size_t i = 0; i < points_set.size(); ++i) {
        Eigen::Vector2i points_i = points_set.at(i);
        auto map_pt = source.points_[points_i[0]];
        auto gt_pt = target.points_[points_i[1]];
        double norm_dis = (map_pt - gt_pt).norm();
        double squre_dis = (map_pt - gt_pt).squaredNorm();
        est_gt_dis.at(i) = norm_dis;
        // 20 cm
        if (norm_dis <= param_.trunc_dist_[0]) {
            mean_vec[0] += norm_dis;
            rmse_vec[0] += squre_dis;
            number_vec[0]++;
        }
        if (norm_dis <= param_.trunc_dist_[1]) {
            mean_vec[1] += norm_dis;
            rmse_vec[1] += squre_dis;
            number_vec[1]++;
        }
        if (norm_dis <= param_.trunc_dist_[2]) {
            mean_vec[2] += norm_dis;
            rmse_vec[2] += squre_dis;
            number_vec[2]++;
        }
        if (norm_dis <= param_.trunc_dist_[3]) {
            mean_vec[3] += norm_dis;
            rmse_vec[3] += squre_dis;
            number_vec[3]++;
        }
        if (norm_dis <= param_.trunc_dist_[4]) {
            mean_vec[4] += norm_dis;
            rmse_vec[4] += squre_dis;
            number_vec[4]++;
        }
    }
    mean_vec /= points_set.size();
    rmse_vec /= points_set.size();
//    int source_num = source.points_.size();
    int target_num = source.points_.size();
    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
        fitness_vec[i] = number_vec[i] * 1.0 / target_num;
        rmse_vec[i] = sqrt(rmse_vec[i]);
        double sigma = 0.0;
        for (int j = 0; j < points_set.size(); ++j) {
            double error_dis = est_gt_dis.at(j) - mean_vec[i];
            sigma += std::pow(error_dis, 2);
        }
        sigma /= points_set.size();
        sigma_vec[i] = sqrt(sigma);
    }
    result.push_back(mean_vec);
    result.push_back(rmse_vec);
    result.push_back(fitness_vec);
    result.push_back(sigma_vec);
    result.push_back(number_vec);
}

void MapEval::calculateMetrics(pipelines::registration::RegistrationResult &registration_result) {


    // Assuming registration_result, map_3d_, gt_3d_, and param_ are available
    // Calculate RMSE, mean error, standard deviation, fitness score
    // Implement getDiffRegResult to compute these metrics
    TicToc ticToc;
    vector<double> est_gt_dis(registration_result.correspondence_set_.size(), 0.0);
    getDiffRegResult(est_gt_results,
                     registration_result.correspondence_set_,
                     *map_3d_, *gt_3d_, *corresponding_cloud_est, *corresponding_cloud_gt);
    std::cout << "5. calculate est-gt metrics : " << ticToc.toc() / 1000.0 << " [s]"
              << std::endl;
    t_acc = ticToc.toc() / 1000.0;

    //getDiffRegResult(est_gt_results, registration_result.correspondence_set_, *map_3d_, *gt_3d_);
    std::cout << "RMSE: " << est_gt_results.at(1).transpose() << std::endl;
    std::cout << "Mean error: " << est_gt_results.at(0).transpose() << std::endl;
    std::cout << "Standard deviation: " << est_gt_results.at(3).transpose() << std::endl;
    std::cout << "Fitness: " << est_gt_results.at(2).transpose() << std::endl;
    // F1 Score and Chamfer Distance
    f1_vec = Vector5d::Zero();
    auto gt_est_icp_results = pipelines::registration::EvaluateRegistration(*gt_3d_, *map_3d_,
                                                                            param_.icp_max_distance_);
    getDiffRegResult(gt_est_results, gt_est_icp_results.correspondence_set_, *gt_3d_, *map_3d_);
    cd_vec = est_gt_results.at(1) + gt_est_results.at(1);  // Chamfer Distance
    iou_vec = Vector5d::Zero();

//    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
//        double pr = 2 * est_gt_results.at(2)[i] * gt_est_results.at(2)[i];
//        double p_plus_r = est_gt_results.at(2)[i] + gt_est_results.at(2)[i];
//        f1_vec[i] = pr / p_plus_r;  // F1 Score
//    }

    // Calculate IoU
//    iou_vec = Vector5d::Zero();
//    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
//        int num_intersection = est_gt_results.at(4)[i];
////        int num_intersection = 0;
////        for (size_t j = 0; j < registration_result.correspondence_set_.size(); ++j) {
////            if (est_gt_dis[j] <= param_.trunc_dist_[i]) {
////                num_intersection++;
////            }
////        }
//        int num_union = map_3d_->points_.size() + gt_3d_->points_.size() - num_intersection;
//        iou_vec[i] = static_cast<double>(num_intersection) / num_union;
//    }

    // F1 Score and Chamfer Distance
    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
        double overlap_ratio = est_gt_results.at(2)[i];
        double rmse = est_gt_results.at(1)[i];
        f1_vec[i] = 2 * overlap_ratio * rmse / (overlap_ratio + rmse);  // F1 Score
        int num_intersection = est_gt_results.at(4)[i];
        int num_union = map_3d_->points_.size() + gt_3d_->points_.size() - num_intersection;
        iou_vec[i] = static_cast<double>(num_intersection) / num_union;
    }

    // std::cout << "Chamfer Distance: " << cd_vec.transpose() << std::endl;
    // std::cout << "F1 Score: " << f1_vec.transpose() << std::endl;
    std::cout << "MME: " << mme_est << " " << mme_gt << std::endl;
    // std::cout << "IoU: " << iou_vec.transpose() << std::endl;

    //计算并打印原始点云的chamfer distance
    TicToc ticToc1;
    full_chamfer_dist = computeChamferDistance(*map_3d_, *gt_3d_);
    t_fcd = ticToc1.toc() / 1000.0;
    std::cout << "Full point cloud Chamfer distance: " << full_chamfer_dist << std::endl;
    std::cout << "5. calculate full cd metrics : " << t_fcd << " [s]" << std::endl;
}

void MapEval::calculateMetricsWithInitialMatrix() {
    // 使用初始矩阵将map_3d_变换到gt_3d_的坐标系下
    *map_3d_ = map_3d_->Transform(param_.initial_matrix_);
    // 手动计算对应点
    // 种方式假设给定的初始矩阵已经足够精确,可以直接用于计算map metric。如果初始矩阵存在较大误差,计算出的指标可能不够准确。
    pipelines::registration::CorrespondenceSet est_gt_correspondence;
    pipelines::registration::CorrespondenceSet gt_est_correspondence;

    // 计算est_gt_correspondence
    open3d::geometry::KDTreeFlann kdtree_gt;
    kdtree_gt.SetGeometry(*gt_3d_);
    for (size_t i = 0; i < map_3d_->points_.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<double> distances(1);
        if (kdtree_gt.SearchKNN(map_3d_->points_[i], 1, indices, distances) > 0) {
            if (distances[0] <= param_.icp_max_distance_) {
                est_gt_correspondence.push_back(Eigen::Vector2i(i, indices[0]));
            }
        }
    }
    // 计算gt_est_correspondence
    open3d::geometry::KDTreeFlann kdtree_est;
    kdtree_est.SetGeometry(*map_3d_);
    for (size_t i = 0; i < gt_3d_->points_.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<double> distances(1);
        if (kdtree_est.SearchKNN(gt_3d_->points_[i], 1, indices, distances) > 0) {
            if (distances[0] <= param_.icp_max_distance_) {
                gt_est_correspondence.push_back(Eigen::Vector2i(indices[0], i));
            }
        }
    }
    // 计算est_gt_results
    //    getDiffRegResult(est_gt_results, est_gt_correspondence, *map_3d_, *gt_3d_, *corresponding_cloud_est, *corresponding_cloud_gt);
    //    // 计算gt_est_results
    //    getDiffRegResult(gt_est_results, gt_est_correspondence, *gt_3d_, *map_3d_);

    // 计算est_gt_results
    getDiffRegResultWithCorrespondence(est_gt_results, est_gt_correspondence, *map_3d_, *gt_3d_,
                                       *corresponding_cloud_est, *corresponding_cloud_gt);
    // 计算gt_est_results
    getDiffRegResultWithCorrespondence(gt_est_results, gt_est_correspondence, *gt_3d_, *map_3d_,
                                       *corresponding_cloud_est, *corresponding_cloud_gt);



    // 计算Chamfer Distance、F1 Score和IoU
    cd_vec = est_gt_results.at(1) + gt_est_results.at(1);
    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
        double overlap_ratio = est_gt_results.at(2)[i];
        double rmse = est_gt_results.at(1)[i];
        f1_vec[i] = 2 * overlap_ratio * rmse / (overlap_ratio + rmse);
        int num_intersection = est_gt_results.at(4)[i];
        int num_union = map_3d_->points_.size() + gt_3d_->points_.size() - num_intersection;
        iou_vec[i] = static_cast<double>(num_intersection) / num_union;
    }
    // 输出结果
    std::cout << "Chamfer Distance: " << cd_vec.transpose() << std::endl;
    // std::cout << "F1 Score: " << f1_vec.transpose() << std::endl;
    std::cout << "est-gt MME: " << mme_est << " " << mme_gt << std::endl;
    // std::cout << "IoU: " << iou_vec.transpose() << std::endl;
}


void MapEval::saveResults() {

    // 根据pcd_file_name确定子文件夹名称
    std::string subfolder;
    if (param_.pcd_file_name_ == "map.pcd") {
        subfolder = "map_results/";
    } else if (param_.pcd_file_name_ == "final_map_lidar.pcd") {
        subfolder = "final_map_lidar_results/";
    } else {
        std::cerr << "Invalid PCD file name: " << param_.pcd_file_name_ << std::endl;
    }

    // 创建子文件夹(如果不存在)
    std::string results_subfolder = param_.evaluation_map_pcd_path_ + subfolder;
    std::cout << "Save results in " << results_subfolder << std::endl;
    if (!fs::exists(results_subfolder)) {
        fs::create_directory(results_subfolder);
    }
// 将评估结果保存到对应的子文件夹中
    std::string results_file_path = results_subfolder + "map_results.txt";
    std::ofstream file_result(results_file_path, std::ios::app);

    // Save evaluation results to a file
    //    std::string result_file_path = param_.evaluation_map_pcd_path_ + "eval_result.txt";
    //    std::ofstream file_result(result_file_path.c_str(), std::ios::app);
    // Assuming that est_gt_results and other necessary data are available
    // Save point clouds or meshes if required


    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // Format the time as a string (you can adjust the format as needed)
    std::stringstream time_stream;
    time_stream << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X"); // Example format: YYYY-MM-DD HH:MM:SS
    // Output name and current time to the file
    file_result << param_.name_ << " " << time_stream.str() << std::endl;
    file_result << param_.map_gt_path_ << std::endl;
    file_result << param_.evaluation_map_pcd_path_ + param_.pcd_file_name_ << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Inlier RMSE: "
                << est_gt_results.at(1).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Inlier Mean: "
                << est_gt_results.at(0).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Inlier Standard deviation: "
                << est_gt_results.at(3).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Inlier Completeness: "
                << est_gt_results.at(2).transpose() << std::endl;
    file_result << std::fixed << setprecision(5) << "CD: " << cd_vec.transpose()
                << std::endl;
    // file_result << std::fixed << setprecision(5) << "F1: " << f1_vec.transpose()
    //             << std::endl;
    // file_result << std::fixed << setprecision(5) << "IOU: " << iou_vec.transpose()
    //             << std::endl;

    if (param_.evaluate_mme_) {
        file_result << std::fixed << setprecision(5) << "MME: " << mme_est << " " << mme_gt << " "
                    << min_abs_entropy << " " << max_abs_entropy << std::endl;
    }
    file_result << "Time: " << t1 / 1000.0 << " " << (t2 - t1) / 1000.0 << " "
                << (t3 - t2) / 1000.0 << " " << (t4 - t3) / 1000.0 << " "
                << (t5 - t4) / 1000.0 << " " << (t7 - t5) / 1000.0 << std::endl;
    file_result.close();
    std::cout << "Results saved to " << results_file_path << std::endl;

    // rendering distance map
    map_3d_render_inlier = renderDistanceOnPointCloud(
            corresponding_cloud_gt, corresponding_cloud_est, param_.trunc_dist_[0]);
    map_3d_render_raw =
            renderDistanceOnPointCloud(gt_3d_, map_3d_, param_.trunc_dist_[0]);
    //    visualization::DrawGeometries({map_3d_render_inlier}, "mesh result");

    open3d::io::WritePointCloud(results_subfolder + "raw_rendered_dis_map.pcd",
                                *map_3d_render_raw);
    std::cout << "Save rendered inlier distance error map to " << results_subfolder + "raw_rendered_dis_map.pcd"
              << std::endl;
    open3d::io::WritePointCloud(results_subfolder + "inlier_rendered_dis_map.pcd", *map_3d_render_inlier);
    std::cout << "Save rendered raw distance error map to " << results_subfolder + "inlier_rendered_dis_map.pcd"
              << std::endl;


    if (param_.evaluate_mme_) {
        open3d::io::WritePointCloud(results_subfolder + "map_entropy.pcd", *map_3d_entropy);
        std::cout << "Save rendered entropy map to "
                  << results_subfolder + "map_entropy.pcd" << std::endl;
        if (param_.evaluate_gt_mme_) {
            open3d::io::WritePointCloud(results_subfolder + "gt_entropy.pcd", *gt_3d_entropy);
            std::cout << "Save rendered entropy gt map to " << results_subfolder + "gt_entropy.pcd" << std::endl;
            visualization::DrawGeometries({gt_3d_entropy}, "entropy result");
        }
//        visualization::DrawGeometries({map_3d_entropy}, "entropy result");
    }
    // std::cout << "Results saved to " << result_file_path << std::endl;

    // ToDO: can not save mesh file successfully
    if (eva_mesh) {
        // create corresspondence mesh
        shared_ptr<Mesh> correspdence_mesh(new Mesh());
        if (1) {
            corresponding_cloud_est->EstimateNormals(
                    geometry::KDTreeSearchParamHybrid(1.0, 30));
            //correspdence_mesh = createMeshFromPCD(corresponding_cloud_est, 0.6, 11);
        }
        // visualization::DrawGeometries({correspdence_mesh}, "correspdence_mesh");

        // TODO : render mesh by distance error
        std::cout << "render correspondence mesh begin!" << std::endl;
        // gt mesh only save once
        open3d::io::WriteTriangleMesh(results_subfolder + "gt_mesh.ply", *gt_mesh);
        open3d::io::WriteTriangleMesh(results_subfolder + "est_mesh.ply", *est_mesh);
        open3d::io::WriteTriangleMesh(results_subfolder + "correspondence_mesh.ply", *correspdence_mesh);
    }
    // visualization::DrawGeometries({map_3d_render_raw}, "mesh result");
}

pipelines::registration::RegistrationResult MapEval::performICPRegistration() {
    pipelines::registration::RegistrationResult registration_result;
    auto criteria = pipelines::registration::ICPConvergenceCriteria();
    switch (param_.evaluation_method_) {
        case 0:  // point-to-point ICP
            registration_result = pipelines::registration::RegistrationICP(
                    *map_3d_, *gt_3d_, param_.icp_max_distance_, param_.initial_matrix_,
                    pipelines::registration::TransformationEstimationPointToPoint(), criteria);
            break;
        case 1:  // Point-to-plane
            registration_result = pipelines::registration::RegistrationICP(
                    *map_3d_, *gt_3d_, param_.icp_max_distance_, param_.initial_matrix_,
                    pipelines::registration::TransformationEstimationPointToPlane(), criteria);
            break;
        case 2:  // Generalized ICP
            // std::cout << "Using GICP!" << std::endl;
            registration_result = pipelines::registration::RegistrationGeneralizedICP(
                    *map_3d_, *gt_3d_, param_.icp_max_distance_, param_.initial_matrix_,
                    pipelines::registration::TransformationEstimationForGeneralizedICP(), criteria);
            break;
        default:
            std::cout << "Invalid registration type specified" << std::endl;
            break;
    }
    // Transform the map based on ICP results
    trans = registration_result.transformation_;
    *map_3d_ = map_3d_->Transform(trans);
    return registration_result;
}


// 新增：计算两个点云之间的chamfer distance
double MapEval::computeChamferDistance(const geometry::PointCloud &cloud1,
                                           const geometry::PointCloud &cloud2) {
    // 构建KD树用于最近邻搜索
    geometry::KDTreeFlann kdtree1(cloud1);
    geometry::KDTreeFlann kdtree2(cloud2);

    const size_t N = cloud1.points_.size();
    const size_t M = cloud2.points_.size();

    double sum_p_to_q = 0.0;
    double sum_q_to_p = 0.0;

    // cloud1到cloud2的距离
#pragma omp parallel for reduction(+:sum_p_to_q)
    for (int i = 0; i < cloud1.points_.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<double> distances(1);
        if (kdtree2.SearchKNN(cloud1.points_[i], 1, indices, distances) > 0) {
            sum_p_to_q += std::sqrt(distances[0]);
        }
    }
    // cloud2到cloud1的距离
#pragma omp parallel for reduction(+:sum_q_to_p)
    for (int i = 0; i < cloud2.points_.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<double> distances(1);
        if (kdtree1.SearchKNN(cloud2.points_[i], 1, indices, distances) > 0) {
            sum_q_to_p += std::sqrt(distances[0]);
        }
    }
    // 计算平均距离作为chamfer distance
    double cd_dis = sum_p_to_q / N + sum_q_to_p / M;
    return cd_dis;
}

double MapEval::ComputeEntropy(const Eigen::Matrix3d &covariance) {
    double entropy = 0.5 * std::log(2 * M_PI * M_E * covariance.determinant());
    return entropy;
}

double
MapEval::ComputeMeanMapEntropy(const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
                                   std::vector<double> &entropies, double radius) {
    mean_entropy = 0.0;
    valid_points = 0;
    entropies = std::vector<double>(pointcloud->points_.size(), 0.0);
    valid_entropy_points.resize(pointcloud->points_.size(), false);
    StartProcessing(static_cast<int>(pointcloud->points_.size()));

    open3d::geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*pointcloud);

    for (size_t i = 0; i < pointcloud->points_.size(); ++i) {
        std::vector<int> indices;
        std::vector<double> distances;
        if (kdtree.SearchRadius(pointcloud->points_[i], radius, indices, distances) > 0) {
            // Remove the first index since it is the point itself
            indices.erase(indices.begin());
            distances.erase(distances.begin());
            if (indices.size() < 5) {
                // Not enough points to compute covariance matrix
                continue;
            }

            Eigen::MatrixXd points(3, indices.size());
            for (size_t j = 0; j < indices.size(); ++j) {
                points.col(j) = pointcloud->points_[indices[j]].cast<double>();
            }
            Eigen::Vector3d mean = points.rowwise().mean();
            Eigen::MatrixXd centered = points.colwise() - mean;
            Eigen::Matrix3d covariance = (centered * centered.transpose()) / static_cast<double>(indices.size() - 1);
            double entropy = ComputeEntropy(covariance);

            if (!std::isnan(entropy) && !std::isinf(entropy)) {
                mean_entropy += entropy;
                entropies[i] = entropy;
                valid_entropy_points[i] = true;
                ++valid_points;
            }
        }
    }


/*    // Parallelize this loop with OpenMP
#pragma omp parallel for reduction(+:mean_entropy, valid_points)
    for (int i = 0; i < static_cast<int>(pointcloud->points_.size()); ++i) {
        std::vector<int> indices;
        std::vector<double> distances;
        if (kdtree.SearchRadius(pointcloud->points_[i], radius, indices, distances) > 0) {
            // Remove the first index since it is the point itself
            indices.erase(indices.begin());
            distances.erase(distances.begin());
            if (indices.size() < 5) {
                // Not enough points to compute covariance matrix
                continue;
            }
            Eigen::MatrixXd points(3, indices.size());
            for (size_t j = 0; j < indices.size(); ++j) {
                points.col(j) = pointcloud->points_[indices[j]].cast<double>();
            }
            Eigen::Vector3d mean = points.rowwise().mean();
            Eigen::MatrixXd centered = points.colwise() - mean;
            Eigen::Matrix3d covariance = (centered * centered.transpose()) / static_cast<double>(indices.size() - 1);
            double entropy = ComputeEntropy(covariance);
            //std::cout << "covariance determinant : " << covariance.determinant() << std::endl;
//            std::cout << "entropy: " << entropy << std::endl;
            if (!std::isnan(entropy) && !std::isinf(entropy)) {
                mean_entropy += entropy;
                entropies[i] = entropy;
                valid_entropy_points[i] = true;
                ++valid_points;
            }
        }
        int current_processed = ++processed_points;
        if (current_processed % 500000 == 0) {
            using namespace std::chrono;
            auto now = steady_clock::now();
            auto elapsed = duration_cast<duration<double>>(now - start_time).count();
            double average_time_per_point = elapsed / current_processed;
            int points_remaining = total_points - current_processed;
            double estimated_time_remaining = average_time_per_point * points_remaining;
#pragma omp critical
            {
                std::cout << "Processed " << current_processed * 100.0 / total_points << "%. ";
                std::cout << "Estimated time remaining: " << estimated_time_remaining << " seconds." << std::endl;
            }
        }
    }
    */

    if (valid_points > 0) {
        mean_entropy /= valid_points;
    }
    std::cout << "GT MME Valid_points " << valid_points * 100.0 / total_points << "% " << valid_points << " "
              << total_points << std::endl;
    return mean_entropy;
}


double MapEval::ComputeMeanMapEntropyUsingNormal(
        const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
        std::vector<double> &entropies,
        double radius) {
    double mean_entropy = 0.0;
    int valid_points = 0;
    int processed_points = 0;
    const int total_points = static_cast<int>(pointcloud->points_.size());
    entropies = std::vector<double>(pointcloud->points_.size(), 0.0);
    valid_entropy_points.resize(pointcloud->points_.size(), false);
    StartProcessing(total_points);
    // Build KD-Tree
    open3d::geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*pointcloud);
    auto start_time = std::chrono::steady_clock::now();
#pragma omp parallel for reduction(+:mean_entropy, valid_points)
    for (int i = 0; i < total_points; ++i) {
        std::vector<int> indices;
        std::vector<double> distances;
        if (kdtree.SearchRadius(pointcloud->points_[i], radius, indices, distances) > 0) {
            // Remove the query point itself
            indices.erase(indices.begin());
            distances.erase(distances.begin());
            if (indices.size() >= 10) {
                // Build point matrix
                Eigen::MatrixXd points(3, indices.size());
                for (size_t j = 0; j < indices.size(); ++j) {
                    points.col(j) = pointcloud->points_[indices[j]].cast<double>();
                }
                // Compute mean and center
                Eigen::Vector3d mean = points.rowwise().mean();
                Eigen::MatrixXd centered = points.colwise() - mean;
                // Compute covariance matrix
                Eigen::Matrix3d covariance =
                        (centered * centered.transpose()) / static_cast<double>(indices.size() - 1);
                double entropy = ComputeEntropy(covariance);
                if (!std::isnan(entropy) && !std::isinf(entropy)) {
                    mean_entropy += entropy;
                    entropies[i] = entropy;
                    valid_entropy_points[i] = true;
                    ++valid_points;
                }
            }
        }
        // Update progress
#pragma omp atomic
        ++processed_points;
        if (processed_points % 1000000 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time).count();
            double average_time_per_point = elapsed / processed_points;
            int points_remaining = total_points - processed_points;
            double estimated_time_remaining = average_time_per_point * points_remaining;
            std::cout << "Processed " << processed_points * 100.0 / total_points << "%. ";
            std::cout << "Estimated time remaining: " << estimated_time_remaining << " seconds." << std::endl;
        }
    }
    // Compute final mean entropy
    if (valid_points > 0) {
        mean_entropy /= valid_points;
    }
    std::cout << "MME Valid_points " << valid_points * 100.0 / total_points << "% " << valid_points << " "
              << total_points << std::endl;
    // make sure there exists encough valid points for mme caculation
    if (valid_points * 100.0 / total_points < 0.6)
    {
       std::cerr << "valid points is too small, please check the input point cloud" << std::endl;
    }
    return mean_entropy;
}


void MapEval::StartProcessing(int total) {
    start_time = std::chrono::steady_clock::now();
    total_points = total;
}

void MapEval::addGaussianNoise(std::shared_ptr<open3d::geometry::PointCloud> &cloud,
                                   double noise_std_dev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, noise_std_dev);
    for (auto &point : cloud->points_) {
        point(0) += dist(gen);
        point(1) += dist(gen);
        point(2) += dist(gen);
    }
}

void MapEval::addNonUniformDensity(std::shared_ptr<open3d::geometry::PointCloud> &cloud,
                                       double sparse_ratio,
                                       double dense_ratio,
                                       double region_size) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::vector<Eigen::Vector3d> new_points;
    // 遍历所有点
    for (const auto &point : cloud->points_) {
        // 基于点的位置决定是否保留
        // 使用正弦函数创建周期性的稀疏/密集区域
        double x_norm = std::sin(point(0) / region_size * M_PI);
        double y_norm = std::sin(point(1) / region_size * M_PI);
        double threshold;
        // 根据位置决定使用哪个保留比例
        if (x_norm * y_norm > 0) {
            threshold = sparse_ratio;
        } else {
            threshold = dense_ratio;
        }
        // 随机决定是否保留该点
        if (dist(gen) < threshold) {
            new_points.push_back(point);
        }
    }
    cloud->points_ = new_points;
}

void MapEval::addSparseOutliers(std::shared_ptr<open3d::geometry::PointCloud> &cloud,
                                    double outlier_ratio,
                                    double outlier_range) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::normal_distribution<double> normal_dist(0.0, outlier_range);
    int num_outliers = static_cast<int>(cloud->points_.size() * outlier_ratio);
    // 添加离群点
    for (int i = 0; i < num_outliers; ++i) {
        // 随机选择一个基准点
        int base_idx = static_cast<int>(dist(gen) * cloud->points_.size());
        Eigen::Vector3d base_point = cloud->points_[base_idx];
        // 在各个方向上添加较大的随机偏移
        Eigen::Vector3d outlier = base_point;
        outlier(0) += normal_dist(gen);
        outlier(1) += normal_dist(gen);
        outlier(2) += normal_dist(gen);
        cloud->points_.push_back(outlier);
    }
}

void MapEval::addLocalDeformation(std::shared_ptr<open3d::geometry::PointCloud> &cloud,
                                      double deform_radius,
                                      double deform_strength,
                                      Eigen::Vector3d center) {
    // 对每个点进行局部变形
    for (auto &point : cloud->points_) {
        double dist = computeDistance(point, center);
        if (dist < deform_radius) {
            // 使用平滑的衰减函数
            double weight = 0.5 * (1 + std::cos(M_PI * dist / deform_radius));
            // 创建径向变形
            Eigen::Vector3d direction = point - center;
            direction.normalize();
            // 应用变形
            point += direction * deform_strength * weight;
        }
    }
}

double MapEval::computeDistance(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
    return (p1 - p2).norm();
}
