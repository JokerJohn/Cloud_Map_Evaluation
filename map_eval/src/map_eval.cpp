#include "map_eval.h"


int MapEval::process() {
    TicToc tic_toc;
    io::ReadPointCloudOption io_params("auto", true, true, true);

    // Load ground truth point cloud
    std::string file_extension = param_.map_gt_path_.substr(param_.map_gt_path_.find_last_of(".") + 1);
    if (file_extension == "pcd") {
        io::ReadPointCloudFromPCD(param_.map_gt_path_, *gt_3d_, io_params);
    } else if (file_extension == "ply") {
        io::ReadPointCloudFromPLY(param_.map_gt_path_, *gt_3d_, io_params);
    } else {
        std::cerr << "ERROR: Unsupported ground truth file format: " << param_.map_gt_path_ << std::endl;
        return -1;
    }

    // Experimental mode: Add noise to ground truth for simulation
    if (param_.evaluate_noised_gt_) {
        std::cout << "INFO: Adding noise to ground truth point cloud for simulation." << std::endl;

        // Create noisy copy of the ground truth point cloud
        auto noised_gt_3d_raw = std::make_shared<open3d::geometry::PointCloud>(*gt_3d_);
        Eigen::Vector3d center;
        int noise_type_ = 1;

        // Apply different noise types based on experimental settings
        switch (noise_type_) {
            case 0:
                addNonUniformDensity(noised_gt_3d_raw, 0.1, 0.9, 2.0);  // Density variation
                break;
            case 1:
                addSparseOutliers(noised_gt_3d_raw, 0.1, 1);  // Outlier addition
                break;
            case 2:
                center = noised_gt_3d_raw->GetCenter();
                addLocalDeformation(noised_gt_3d_raw, 5.0, 0.5, center);  // Local deformation
                break;
            case 3:
                addGaussianNoise(noised_gt_3d_raw, param_.noise_std_dev_);  // Gaussian noise
                break;
        }
        map_3d_ = noised_gt_3d_raw;
        param_.initial_matrix_.setIdentity();
    } else {
        // Normal mode: Load the evaluation point cloud
        bool success = io::ReadPointCloudFromPCD(param_.evaluation_map_pcd_path_ + param_.pcd_file_name_, *map_3d_, io_params);
        std::cout << "INFO: Loading map point cloud from: " << param_.evaluation_map_pcd_path_ + param_.pcd_file_name_ << std::endl;
        if (!success) {
            std::cerr << "ERROR: Failed to load point cloud from the specified path." << std::endl;
            return -1;
        }
    }

    // Check if the point clouds are empty
    if (map_3d_->IsEmpty() || gt_3d_->IsEmpty()) {
        std::cerr << "ERROR: One or both point clouds are empty!" << std::endl;
        return -1;
    }

    // Downsample the point clouds for efficiency
    map_3d_ = map_3d_->VoxelDownSample(0.01);
    gt_3d_ = gt_3d_->VoxelDownSample(0.01);
    //    open3d::io::WritePointCloud(resul bvts_subfolder + "gt_map_1cm.pcd",  *gt_3d_);


    // Log the number of points in both point clouds
    file_result << std::fixed << std::setprecision(15)
                << "Estimated-Ground Truth point count: "
                << map_3d_->points_.size() << " / " << gt_3d_->points_.size() << std::endl;
    std::cout << "INFO: Loaded point clouds: " << map_3d_->points_.size() << " points (Map), "
              << gt_3d_->points_.size() << " points (Ground Truth)." << std::endl;

    // Perform MME calculation if enabled
    if (param_.evaluate_mme_) {
        std::cout << "INFO: Starting MME calculation..." << std::endl;
        computeMME(map_3d_, gt_3d_);
        std::cout << "INFO: MME calculation completed. Saving results." << std::endl;
        if (param_.save_immediate_result_) saveMmeResults();
            // Log the MME calculation time
        t2 = tic_toc.toc();
        std::cout << "INFO: MME calculation completed in: " << (t2 - t1) / 1000.0 << " seconds." << std::endl;
    }

    // Start registration process
    std::cout << "INFO: Starting registration..." << std::endl;
    if (param_.evaluate_using_initial_) {
        std::cout << "INFO: Using initial matrix without registration." << std::endl;
        calculateMetricsWithInitialMatrix();
    } else {
        std::cout << "INFO: Using ICP for registration." << std::endl;
        performRegistration();
    }

    // Calculate additional metrics
    calculateVMD();
    std::cout << "INFO: VMD calculation completed." << std::endl;

    // Save registration results
    if (param_.save_immediate_result_) {
        std::cout << "INFO: Saving registration results..." << std::endl;
        saveRegistrationResults();
    }

    std::cout << "INFO: Results saved successfully." << std::endl;
    return 0;
}

void MapEval::computeMME() {
    if (param_.evaluate_mme_) {
        TicToc tic_toc;
        // Compute Mean Map Entropy using normal-based method
        mme_est = ComputeMeanMapEntropyUsingNormal(map_3d_, est_entropies, param_.nn_radius_);

        // Preserving commented-out lines for future use
        // Merge entropy vectors from both point clouds (for coloring)
        // std::vector<double> combined_entropies = est_entropies;  
        // combined_entropies.insert(combined_entropies.end(), gt_entropies.begin(), gt_entropies.end());
        // std::vector<double> non_zero_combined;
        // std::copy_if(combined_entropies.begin(), combined_entropies.end(), std::back_inserter(non_zero_combined),
        //              [](double val) { return val != 0.0; });
        // max_abs_entropy = std::fabs(*std::min_element(non_zero_combined.begin(), non_zero_combined.end()));
        // min_abs_entropy = std::fabs(*std::max_element(non_zero_combined.begin(), non_zero_combined.end()));
        // std::cout << "MAX MIN ENTROPY: " << max_abs_entropy << " " << min_abs_entropy << std::endl;

        if (param_.evaluate_gt_mme_) {
            std::cout << "INFO: Calculating GT MME..." << std::endl;
            mme_gt = ComputeMeanMapEntropy(gt_3d_, gt_entropies, param_.nn_radius_);
            std::cout << "MME EST-GT: " << mme_est << " " << mme_gt << std::endl;
        } else {
            std::cout << "MME EST: " << mme_est << std::endl;
        }

        // Color point clouds based on entropy values
        map_3d_entropy = ColorPointCloudByMME(map_3d_, est_entropies);
        if (param_.evaluate_gt_mme_) {
            // No need to use common max/min values after scaling
            gt_3d_entropy = ColorPointCloudByMME(gt_3d_, gt_entropies);
        }

        // Log computation time for MME
        std::cout << "INFO: MME Calculation Time: " << (tic_toc.toc() - t2) / 1000.0 << " [s]" << std::endl;
    }
}

void MapEval::computeMME(shared_ptr<PointCloud> &cloud_, shared_ptr<PointCloud> &cloud_gt_) {
    if (param_.evaluate_mme_) {
        TicToc tic_toc;
        // Compute Mean Map Entropy for the given cloud
        mme_est = ComputeMeanMapEntropyUsingNormal(cloud_, est_entropies, param_.nn_radius_);

        // Preserving commented-out lines for future use
        // Merge entropy vectors for proper scaling (for coloring)
        // std::vector<double> combined_entropies = est_entropies;
        // combined_entropies.insert(combined_entropies.end(), gt_entropies.begin(), gt_entropies.end());
        // std::vector<double> non_zero_combined;
        // std::copy_if(combined_entropies.begin(), combined_entropies.end(), std::back_inserter(non_zero_combined),
        //              [](double val) { return val != 0.0; });
        // max_abs_entropy = std::fabs(*std::min_element(non_zero_combined.begin(), non_zero_combined.end()));
        // min_abs_entropy = std::fabs(*std::max_element(non_zero_combined.begin(), non_zero_combined.end()));
        // std::cout << "MAX MIN ENTROPY: " << max_abs_entropy << " " << min_abs_entropy << std::endl;

        if (param_.evaluate_gt_mme_) {
            // Compute and log GT MME
            mme_gt = ComputeMeanMapEntropy(cloud_gt_, gt_entropies, param_.nn_radius_);
            std::cout << "MME EST-GT: " << mme_est << " " << mme_gt << std::endl;
        } else {
            std::cout << "MME EST: " << mme_est << std::endl;
        }

        // Color point clouds based on entropy values
        map_3d_entropy = ColorPointCloudByMME(cloud_, est_entropies);
        if (param_.evaluate_gt_mme_) {
            gt_3d_entropy = ColorPointCloudByMME(cloud_gt_, gt_entropies);
        }

        // Log MME computation time
        std::cout << "INFO: MME Calculation Time: " << (tic_toc.toc() - t2) / 1000.0 << " [s]" << std::endl;
    }
}

void MapEval::performRegistration() {
    TicToc tic_toc;
    
    // Check if mesh creation is enabled
    if (eva_mesh) {
        // Create meshes for ground truth and estimated point clouds
        gt_mesh = createMeshFromPCD(gt_3d_, 0.6, 10);
        
        // Preserving commented-out line for future mesh filtering
        // gt_mesh_filtered = gt_mesh->FilterSmoothLaplacian(10, 0.5);
        // gt_mesh_filtered->ComputeVertexNormals();
        // gt_mesh_filtered->PaintUniformColor({1, 0.7, 0});
        // visualization::DrawGeometries({gt_mesh_filtered}, "mesh result");

        est_mesh = createMeshFromPCD(map_3d_, 0.6, 10);
        t3 = tic_toc.toc();  // Measure time for mesh creation
        std::cout << "INFO: Mesh creation time: " << (t3) / 1000.0 << " [s]" << std::endl;
    }

    // Perform ICP registration
    auto registration_result = performICPRegistration();
    t4 = tic_toc.toc();  // Measure time for ICP

    std::cout << "INFO: ICP registration time: " << (t4 - t3) / 1000.0 << " [s]" << std::endl;
    std::cout << "INFO: Aligned transformation: \n" << trans << std::endl;
    std::cout << "INFO: ICP overlap ratio: " << registration_result.fitness_ << std::endl;
    std::cout << "INFO: ICP correspondences RMSE: " << registration_result.inlier_rmse_ << std::endl;
    std::cout << "INFO: ICP correspondences size: " << registration_result.correspondence_set_.size() << std::endl;

    // Log ICP results to the output file
    file_result << std::fixed << setprecision(5) << "Aligned cloud: " << trans.matrix() << std::endl;
    file_result << std::fixed << setprecision(5) << "Aligned results: " << registration_result.fitness_ << " "
                << registration_result.correspondence_set_.size() << std::endl;
    std::cout << "INFO: Aligned results saved to " << results_file_path << std::endl;

    // Calculate metrics and evaluate the results
    calculateMetrics(registration_result);
    t5 = tic_toc.toc();  // Measure time for metric calculation
    std::cout << "INFO: Metrics calculation time: " << (t7 - t5) / 1000.0 << " [s]" << std::endl;
}


void MapEval::calculateVMD() {
    TicToc ticToc;

    // Create VoxelCalculator instances for ground truth and estimated maps
    VoxelCalculator gt_calculator(param_.vmd_voxel_size_);
    VoxelCalculator est_calculator(param_.vmd_voxel_size_);
    
    // Build voxel maps for both GT and estimated maps
    gt_calculator.buildVoxelMap(*gt_3d_);
    est_calculator.buildVoxelMap(*map_3d_);
    
    // Update the estimated voxel map with information from the ground truth
    est_calculator.updateVoxelMap(gt_calculator.getVoxelMap());
    t_v = ticToc.toc();  // Timing the process

    // Prepare to write the voxel error results to a file
    std::ofstream output_file(results_subfolder + "voxel_errors.txt");
    if (!output_file.is_open()) {
        std::cerr << "ERROR: Failed to open voxel error output file." << std::endl;
        return;  // Early return on failure to open the file
    }

    const VoxelMap &gt_map = gt_calculator.getVoxelMap();
    const VoxelMap &est_map = est_calculator.getVoxelMap();
    
    // Map to store Wasserstein distances for active voxels
    std::unordered_map<Eigen::Vector3i, double, hash_eigen<Eigen::Vector3i>> wasserstein_distances;

    // Loop over the estimated voxels and calculate Wasserstein distances for active ones
    for (const auto &kv : est_map) {
        const Eigen::Vector3i &index = kv.first;
        const VoxelInfo &est_voxel = kv.second;

        // Only process active voxels
        if (est_voxel.active == 1) {
            auto gt_it = gt_map.find(index);
            if (gt_it != gt_map.end()) {
                const VoxelInfo &gt_voxel = gt_it->second;
                
                // Skip voxels with insufficient points
                if (est_voxel.num_points < 100 || gt_voxel.num_points < 100)
                    continue;

                // Compute the Wasserstein distance for the voxel pair
                double ws_distance = est_calculator.computeWassersteinDistanceGaussian(gt_voxel, est_voxel);
                wasserstein_distances[index] = ws_distance;

                // Calculate voxel boundaries
                Eigen::Vector3d voxel_min = index.cast<double>() * param_.vmd_voxel_size_;
                Eigen::Vector3d voxel_max = (index.cast<double>() + Eigen::Vector3d::Ones()) * param_.vmd_voxel_size_;

                // Output results to file
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
                            << gt_voxel.sigma(2, 2) << std::endl;
            }
        }
    }

    // Close output file after writing
    output_file.close();
    std::cout << "INFO: Voxel errors results saved to " << results_subfolder + "voxel_errors.txt" << std::endl;

    // Calculate the mean Wasserstein distance
    std::vector<double> ws_distances;
    for (const auto &kv : wasserstein_distances) {
        ws_distances.push_back(kv.second);
    }
    //     // exists bugs here, wrong codes according to the eq.9
    // double std_dev_ws = std::sqrt(variance_ws);
    // double w_3sigma = mean_ws + 3 * std_dev_ws;
    // std::cout << "3-Sigma Threshold: " << w_3sigma << std::endl;

    // Calculate mean and report VMD
    double mean_ws = std::accumulate(ws_distances.begin(), ws_distances.end(), 0.0) / ws_distances.size();
    vmd = mean_ws;
    t_vmd = ticToc.toc();  // Timing VMD calculation
    std::cout << "INFO: Calculated VMD: " << mean_ws << std::endl;

    // Optional: Calculate and report CDF of Wasserstein distances
    std::sort(ws_distances.begin(), ws_distances.end());
    std::ofstream cdf_file(results_subfolder + "voxel_wasserstein_cdf.txt");
    if (!cdf_file.is_open()) {
        std::cerr << "ERROR: Failed to open CDF output file." << std::endl;
        return;  // Early return if file can't be opened
    }

    for (size_t i = 0; i < ws_distances.size(); ++i) {
        double cdf_value = static_cast<double>(i + 1) / ws_distances.size();
        cdf_file << ws_distances[i] << " " << cdf_value << std::endl;
    }
    cdf_file.close();
    t_cdf = ticToc.toc();  // Timing CDF calculation
    std::cout << "INFO: CDF results saved to " << results_subfolder + "voxel_wasserstein_cdf.txt" << std::endl;

    // Calculate the Spatial Consistency Score (SCS)
    std::cout << "INFO: Calculating Spatial Consistency Score (SCS)..." << std::endl;
    double total_scs = 0.0;
    int scs_count = 0;
    int radius = 5;  // Radius for neighborhood

    // Compute SCS based on neighboring Wasserstein distances
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

        // Calculate SCS if neighbors are available
        if (!neighbor_ws_distances.empty()) {
            double mean_ws_neighbor = std::accumulate(neighbor_ws_distances.begin(), neighbor_ws_distances.end(), 0.0) / neighbor_ws_distances.size();
            double variance_ws = 0.0;
            for (double w : neighbor_ws_distances) {
                variance_ws += (w - mean_ws_neighbor) * (w - mean_ws_neighbor);
            }
            variance_ws /= neighbor_ws_distances.size();
            double std_dev_ws = std::sqrt(variance_ws);
            double scs = std_dev_ws / mean_ws_neighbor;
            total_scs += scs;
            scs_count++;
        }
    }

    scs_overall = total_scs / scs_count;
    t_scs = ticToc.toc();  // Timing SCS calculation
    std::cout << "INFO: Spatial Consistency Score (SCS): " << scs_overall << std::endl;
}

void MapEval::saveMmeResults() {
    if (param_.evaluate_mme_) {
        file_mt.lock();  // Lock file access to ensure thread safety
        file_result << std::fixed << std::setprecision(5) << "MME: " << mme_est << " " << mme_gt << " "
                    << min_abs_entropy << " " << max_abs_entropy << std::endl;
        file_mt.unlock();  // Unlock file access

        std::cout << "INFO: MME results saved to " << results_file_path << std::endl;

        // Save point clouds
        open3d::io::WritePointCloud(results_subfolder + "map_entropy.pcd", *map_3d_entropy);
        std::cout << "INFO: Saved rendered entropy map to " << results_subfolder + "map_entropy.pcd" << std::endl;

        if (param_.evaluate_gt_mme_) {
            open3d::io::WritePointCloud(results_subfolder + "gt_entropy.pcd", *gt_3d_entropy);
            std::cout << "INFO: Saved rendered entropy ground truth map to " << results_subfolder + "gt_entropy.pcd" << std::endl;
        }
    }
}


void MapEval::saveRegistrationResults() {

    // Log time taken for different stages
    std::cout << "INFO: AC+MME Time: " << t_acc + (t2 - t1) / 1000.0 << std::endl;
    std::cout << "INFO: CD+MME Time: " << t_fcd + (t2 - t1) / 1000.0 << std::endl;
    std::cout << "INFO: AWD+SCS Time: " << t_v / 1000.0 + (t_vmd - t_v) / 1000.0 + (t_scs - t_cdf) / 1000.0 << std::endl;

    // Save file results
    file_mt.lock();
    
    // Log RMSE, Mean, Std, and Compensation results
    file_result << std::fixed << std::setprecision(15) << "RMSE/AC: "
                << est_gt_results.at(1).transpose() << std::endl;
    // file_result << std::fixed << std::setprecision(15) << "Mean: "
    //             << est_gt_results.at(0).transpose() << std::endl;
    // file_result << std::fixed << std::setprecision(15) << "Std: "
    //             << est_gt_results.at(3).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Comp: "
                << est_gt_results.at(2).transpose() << std::endl;
    
    // Preserving commented-out lines for future use
    // file_result << std::fixed << setprecision(5) << "CD: " << cd_vec.transpose()
    //             << std::endl;
    
    // Log full Chamfer Distance
    file_result << std::fixed << setprecision(5) << "FULL CD: " << full_chamfer_dist << std::endl;
    
    // Preserving commented-out lines for future use
    // file_result << std::fixed << std::setprecision(5) << "F1: " << f1_vec.transpose()
    //             << std::endl;
    // file_result << std::fixed << std::setprecision(5) << "IOU: " << iou_vec.transpose()
    //             << std::endl;
    
    // Log VMD and SCS
    file_result << std::fixed << std::setprecision(5) << "VMD: " << vmd << std::endl;
    file_result << std::fixed << std::setprecision(5) << "SCS: " << scs_overall << std::endl;

    // Log timing details for the different stages
    file_result << "Time load-MME-mesh-ICP-Metric-AC-FCD: " << t1 / 1000.0 << " " << (t2 - t1) / 1000.0 << " "
                << (t3) / 1000.0 << " " << (t4 - t3) / 1000.0 << " "
                << (t5 - t4) / 1000.0 << " " << t_acc << " " << t_fcd << std::endl;
    
    file_result << "VMD Time voxelization-WD-CDF-SCS: " << t_v / 1000.0 << " " << (t_vmd - t_v) / 1000.0 << " "
                << (t_cdf - t_vmd) / 1000.0 << " " << (t_scs - t_cdf) / 1000.0 << std::endl;

    // Additional timing logs
    file_result << "AC+MME Time: " << t_acc + (t2 - t1) / 1000.0 << std::endl;
    file_result << "CD+MME Time: " << t_fcd + (t2 - t1) / 1000.0 << std::endl;
    file_result << "AWD+SCS Time: " << t_v / 1000.0 + (t_vmd - t_v) / 1000.0 + (t_scs - t_cdf) / 1000.0 << std::endl;

    file_result.close();
    file_mt.unlock();
    std::cout << "INFO: Results saved to " << results_subfolder + "map_results.txt" << std::endl;

    // Save rendered distance error maps
    map_3d_render_inlier = renderDistanceOnPointCloud(corresponding_cloud_gt, corresponding_cloud_est, param_.trunc_dist_[0]);
    map_3d_render_raw = renderDistanceOnPointCloud(gt_3d_, map_3d_, param_.trunc_dist_[0]);

    // Save the raw and inlier distance maps
    open3d::io::WritePointCloud(results_subfolder + "raw_rendered_dis_map.pcd", *map_3d_render_raw);
    std::cout << "INFO: Saved raw distance error map to " << results_subfolder + "raw_rendered_dis_map.pcd" << std::endl;
    open3d::io::WritePointCloud(results_subfolder + "inlier_rendered_dis_map.pcd", *map_3d_render_inlier);
    std::cout << "INFO: Saved inlier distance error map to " << results_subfolder + "inlier_rendered_dis_map.pcd" << std::endl;

    // If noisy ground truth is evaluated, save the noisy GT map
    if (param_.evaluate_noised_gt_) {
        open3d::io::WritePointCloud(results_subfolder + "noise_gt_map.pcd", *map_3d_);
        std::cout << "INFO: Saved noisy ground truth map to " << results_subfolder + "noise_gt_map.pcd" << std::endl;
    }

    // Save meshes (if mesh evaluation is enabled)
    if (eva_mesh) {
        // Create correspondence mesh if needed
        shared_ptr<Mesh> correspondence_mesh(new Mesh());
        corresponding_cloud_est->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
        
        // Optional: render mesh by distance error
        std::cout << "INFO: Rendering correspondence mesh..." << std::endl;
        
        // Save GT and estimated meshes
        open3d::io::WriteTriangleMesh(results_subfolder + "gt_mesh.ply", *gt_mesh);
        open3d::io::WriteTriangleMesh(results_subfolder + "est_mesh.ply", *est_mesh);
        open3d::io::WriteTriangleMesh(results_subfolder + "correspondence_mesh.ply", *correspondence_mesh);

        std::cout << "INFO: Saved meshes to the following paths:" << std::endl;
        std::cout << "GT Mesh: " << results_subfolder + "gt_mesh.ply" << std::endl;
        std::cout << "Estimated Mesh: " << results_subfolder + "est_mesh.ply" << std::endl;
        std::cout << "Correspondence Mesh: " << results_subfolder + "correspondence_mesh.ply" << std::endl;
    }

    // Optional: Render the geometry for visualization
    visualization::DrawGeometries({map_3d_render_raw}, "Error Raw Map Visualization");
    visualization::DrawGeometries({map_3d_render_inlier}, "Error Correspondence Map Visualization");
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
    TicToc ticToc;

    // Calculate the distances between the estimated and ground truth point clouds
    vector<double> est_gt_dis(registration_result.correspondence_set_.size(), 0.0);
    getDiffRegResult(est_gt_results,  registration_result.correspondence_set_,
                     *map_3d_, *gt_3d_, *corresponding_cloud_est, *corresponding_cloud_gt);

    std::cout << "INFO: Calculating est-gt metrics took: " << ticToc.toc() / 1000.0 << " [s]" << std::endl;
    t_acc = ticToc.toc() / 1000.0;

    // Log calculated metrics: RMSE, Mean, Standard deviation, Fitness score
    std::cout << "INFO: RMSE/AC: " << est_gt_results.at(1).transpose() << std::endl;
    // std::cout << "INFO: Mean error: " << est_gt_results.at(0).transpose() << std::endl;
    // std::cout << "INFO: Standard deviation: " << est_gt_results.at(3).transpose() << std::endl;
    std::cout << "INFO: Fitness/Overlap: " << est_gt_results.at(2).transpose() << std::endl;

    // F1 Score and Chamfer Distance
    f1_vec = Vector5d::Zero();
    auto gt_est_icp_results = pipelines::registration::EvaluateRegistration(*gt_3d_, *map_3d_, param_.icp_max_distance_);
    getDiffRegResult(gt_est_results, gt_est_icp_results.correspondence_set_, *gt_3d_, *map_3d_);
    cd_vec = est_gt_results.at(1) + gt_est_results.at(1);  // Chamfer Distance
    iou_vec = Vector5d::Zero();

    // F1 Score and IoU calculation (commented-out for future use)
    // for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
    //     double pr = 2 * est_gt_results.at(2)[i] * gt_est_results.at(2)[i];
    //     double p_plus_r = est_gt_results.at(2)[i] + gt_est_results.at(2)[i];
    //     f1_vec[i] = pr / p_plus_r;  // F1 Score
    // }

    // Calculate IoU (commented-out for future use)
    // iou_vec = Vector5d::Zero();
    // for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
    //     int num_intersection = est_gt_results.at(4)[i];
    //     int num_union = map_3d_->points_.size() + gt_3d_->points_.size() - num_intersection;
    //     iou_vec[i] = static_cast<double>(num_intersection) / num_union;
    // }

    // Log calculated metrics
    std::cout << "INFO: MME: " << mme_est << " " << mme_gt << std::endl;

    // Calculate and print Chamfer distance
    TicToc ticToc1;
    full_chamfer_dist = computeChamferDistance(*map_3d_, *gt_3d_);
    t_fcd = ticToc1.toc() / 1000.0;
    std::cout << "INFO: Full Chamfer distance: " << full_chamfer_dist << std::endl;
    std::cout << "INFO: Chamfer distance calculation took: " << t_fcd << " [s]" << std::endl;
}

void MapEval::calculateMetricsWithInitialMatrix() {
    // Apply initial matrix transformation to map_3d_
    *map_3d_ = map_3d_->Transform(param_.initial_matrix_);

    // Calculate correspondences using KD-tree for efficiency
    pipelines::registration::CorrespondenceSet est_gt_correspondence;
    pipelines::registration::CorrespondenceSet gt_est_correspondence;

    // Calculate est_gt_correspondence (ground truth -> estimated)
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

    // Calculate gt_est_correspondence (estimated -> ground truth)
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

    // Calculate the metrics (est_gt_results and gt_est_results)
    getDiffRegResultWithCorrespondence(est_gt_results, est_gt_correspondence, *map_3d_, *gt_3d_,
                                       *corresponding_cloud_est, *corresponding_cloud_gt);
    getDiffRegResultWithCorrespondence(gt_est_results, gt_est_correspondence, *gt_3d_, *map_3d_,
                                       *corresponding_cloud_est, *corresponding_cloud_gt);

    // Calculate Chamfer Distance, F1 Score, and IoU (for initial matrix)
    cd_vec = est_gt_results.at(1) + gt_est_results.at(1);
    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
        double overlap_ratio = est_gt_results.at(2)[i];
        double rmse = est_gt_results.at(1)[i];
        f1_vec[i] = 2 * overlap_ratio * rmse / (overlap_ratio + rmse);
        int num_intersection = est_gt_results.at(4)[i];
        int num_union = map_3d_->points_.size() + gt_3d_->points_.size() - num_intersection;
        iou_vec[i] = static_cast<double>(num_intersection) / num_union;
    }

    // Log the results
    std::cout << "INFO: Chamfer Distance: " << cd_vec.transpose() << std::endl;
    std::cout << "INFO: F1 Score: " << f1_vec.transpose() << std::endl;
    std::cout << "INFO: est-gt MME: " << mme_est << " " << mme_gt << std::endl;
    std::cout << "INFO: IoU: " << iou_vec.transpose() << std::endl;
}

void MapEval::saveResults() {
    // Create results subfolder based on the PCD file name
    std::string subfolder;
    if (param_.pcd_file_name_ == "map.pcd") {
        subfolder = "map_results/";
    } else if (param_.pcd_file_name_ == "final_map_lidar.pcd") {
        subfolder = "final_map_lidar_results/";
    } else {
        std::cerr << "ERROR: Invalid PCD file name: " << param_.pcd_file_name_ << std::endl;
        return;  // Exit if the file name is invalid
    }

    // Create subfolder if it doesn't exist
    std::string results_subfolder = param_.evaluation_map_pcd_path_ + subfolder;
    std::cout << "INFO: Saving results in " << results_subfolder << std::endl;
    if (!fs::exists(results_subfolder)) {
        fs::create_directory(results_subfolder);
    }

    // Open result file for appending
    std::string results_file_path = results_subfolder + "map_results.txt";
    std::ofstream file_result(results_file_path, std::ios::app);

    // Write current time and results to file
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream time_stream;
    time_stream << std::put_time(std::localtime(&now_c), "%Y-%m-%d %X");
    file_result << param_.name_ << " " << time_stream.str() << std::endl;
    file_result << param_.map_gt_path_ << std::endl;
    file_result << param_.evaluation_map_pcd_path_ + param_.pcd_file_name_ << std::endl;

    // Save metrics results
    file_result << std::fixed << std::setprecision(15) << "Inlier RMSE: "
                << est_gt_results.at(1).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Inlier Mean: "
                << est_gt_results.at(0).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Inlier Standard deviation: "
                << est_gt_results.at(3).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15) << "Inlier Completeness: "
                << est_gt_results.at(2).transpose() << std::endl;
    file_result << std::fixed << setprecision(5) << "CD: " << cd_vec.transpose() << std::endl;
    
    // Preserving commented-out lines for future use
    // file_result << std::fixed << setprecision(5) << "F1: " << f1_vec.transpose() << std::endl;
    // file_result << std::fixed << setprecision(5) << "IOU: " << iou_vec.transpose() << std::endl;

    if (param_.evaluate_mme_) {
        file_result << std::fixed << setprecision(5) << "MME: " << mme_est << " " << mme_gt << " "
                    << min_abs_entropy << " " << max_abs_entropy << std::endl;
    }

    // Log time taken for each stage
    file_result << "Time: " << t1 / 1000.0 << " " << (t2 - t1) / 1000.0 << " "
                << (t3 - t2) / 1000.0 << " " << (t4 - t3) / 1000.0 << " "
                << (t5 - t4) / 1000.0 << " " << (t7 - t5) / 1000.0 << std::endl;
    file_result.close();
    std::cout << "INFO: Results saved to " << results_file_path << std::endl;

    // Save rendered distance maps
    map_3d_render_inlier = renderDistanceOnPointCloud(corresponding_cloud_gt, corresponding_cloud_est, param_.trunc_dist_[0]);
    map_3d_render_raw = renderDistanceOnPointCloud(gt_3d_, map_3d_, param_.trunc_dist_[0]);
    open3d::io::WritePointCloud(results_subfolder + "raw_rendered_dis_map.pcd", *map_3d_render_raw);
    std::cout << "INFO: Saved rendered inlier distance error map to " << results_subfolder + "raw_rendered_dis_map.pcd" << std::endl;
    open3d::io::WritePointCloud(results_subfolder + "inlier_rendered_dis_map.pcd", *map_3d_render_inlier);
    std::cout << "INFO: Saved rendered raw distance error map to " << results_subfolder + "inlier_rendered_dis_map.pcd" << std::endl;

    // Save meshes and entropy maps if required
    if (param_.evaluate_mme_) {
        open3d::io::WritePointCloud(results_subfolder + "map_entropy.pcd", *map_3d_entropy);
        std::cout << "INFO: Saved rendered entropy map to " << results_subfolder + "map_entropy.pcd" << std::endl;
        if (param_.evaluate_gt_mme_) {
            open3d::io::WritePointCloud(results_subfolder + "gt_entropy.pcd", *gt_3d_entropy);
            std::cout << "INFO: Saved rendered entropy ground truth map to " << results_subfolder + "gt_entropy.pcd" << std::endl;
            visualization::DrawGeometries({gt_3d_entropy}, "entropy result");
        }
    }

    // Mesh saving code (for future use)
    if (eva_mesh) {
        shared_ptr<Mesh> correspondence_mesh(new Mesh());
        corresponding_cloud_est->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
        std::cout << "INFO: Rendering correspondence mesh..." << std::endl;
        open3d::io::WriteTriangleMesh(results_subfolder + "gt_mesh.ply", *gt_mesh);
        open3d::io::WriteTriangleMesh(results_subfolder + "est_mesh.ply", *est_mesh);
        open3d::io::WriteTriangleMesh(results_subfolder + "correspondence_mesh.ply", *correspondence_mesh);
    }

    visualization::DrawGeometries({map_3d_render_raw}, "mesh result");
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
    // need a parallel version
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
