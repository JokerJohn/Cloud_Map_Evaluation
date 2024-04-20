#include "cloud_map_eval.h"

int CloudMapEva::process() {
    TicToc tic_toc;
    io::ReadPointCloudOption io_params("auto", true, true, true);
    // Load point clouds
    io::ReadPointCloudFromPCD(param_.evaluation_map_pcd_path_ + "map.pcd", *map_3d_, io_params);
    io::ReadPointCloudFromPCD(param_.map_gt_path_, *gt_3d_, io_params);
    if (map_3d_->IsEmpty() || gt_3d_->IsEmpty()) {
        cout << "Empty point cloud detected" << endl;
        return -1;
    }
    // Downsampling for efficiency
    map_3d_ = map_3d_->VoxelDownSample(0.01);
    gt_3d_ = gt_3d_->VoxelDownSample(0.01);
    t1 = tic_toc.toc();
    std::cout << "1. load point size: " << map_3d_->points_.size() << ", "
              << gt_3d_->points_.size() << ", " << t1 / 1000.0 << " [s]"
              << std::endl;
    t2 = tic_toc.toc();
    std::cout << "2. estimate normal: " << (t2 - t1) / 1000.0 << " [s]" << std::endl;

    if (param_.evaluate_mme_) {
        double radius = 0.2;
        mme_est = ComputeMeanMapEntropy(map_3d_, est_entropies, radius);

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
            mme_gt = ComputeMeanMapEntropy(gt_3d_, gt_entropies, radius);
            std::cout << "MME EST-GT: " << mme_est << " " << mme_gt << std::endl;
        } else {
            std::cout << "MME EST: " << mme_est << std::endl;
        }
        map_3d_entropy = ColorPointCloudByMME(map_3d_, est_entropies);
        if (param_.evaluate_gt_mme_) {
            /** since we finally scale the range of entropy, no need to use the common max and min value*/
            gt_3d_entropy = ColorPointCloudByMME(gt_3d_, gt_entropies);
        }
        std::cout << "2. Calculate MME: " << (tic_toc.toc() - t2) / 1000.0 << " [s]" << std::endl;
    }

    /** if building mesh */
    if (eva_mesh) {
        gt_mesh = createMeshFromPCD(gt_3d_, 0.6, 10);
        //    gt_mesh_filtered = gt_mesh->FilterSmoothLaplacian(10, 0.5);
        //    gt_mesh_filtered->ComputeVertexNormals();
        //    gt_mesh_filtered->PaintUniformColor({1, 0.7, 0});
        //    visualization::DrawGeometries({gt_mesh_filtered}, "mesh result");
        est_mesh = createMeshFromPCD(map_3d_, 0.6, 10);
    }
    t3 = tic_toc.toc();
    std::cout << "3. create mesh: " << (t3 - t2) / 1000.0 << " [s]" << std::endl;

    // ICP registration
    pipelines::registration::RegistrationResult registration_result;
    registration_result = performICPRegistration();
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

    // Calculate metrics and evaluate
    t5 = tic_toc.toc();
    calculateMetrics(registration_result);
    t7 = tic_toc.toc();
    std::cout << "6. caculate est-gt and gt-est metrics : " << (t7 - t5) / 1000.0 << " [s]"
              << std::endl;
    // Optional: Mesh processing and saving
    if (param_.save_immediate_result_) {
        saveResults(); // Implement this function to save results and perform mesh processing
    }
    return 0;
}

int CloudMapEva::loadTrajTUM(const string &traj_path,
                             vector<Vector8d> &traj_vec) {
    std::ifstream file_reader(traj_path + "optimized_odom_tum.txt");

    std::cout << "load tum traj txt " << traj_path + "optimized_odom_tum.txt"
              << std::endl;
    Vector8d pose;
    if (file_reader.is_open()) {
        int index = 0;
        while (file_reader.peek() != EOF) {
            std::string line;
            std::getline(file_reader, line, '\n');
            std::istringstream line_reader(line);
            line_reader >> pose(0, 0) >> pose(1, 0) >> pose(2, 0) >> pose(3, 0) >> pose(4, 0) >> pose(5, 0)
                        >> pose(6, 0) >> pose(7, 0);
            traj_vec.emplace_back(pose);
            std::cout << "read  tum pose: " << index << ", "
                      << pose.matrix().transpose() << std::endl;
            index++;
        }
    }
    file_reader.close();
}

int CloudMapEva::saveTrajTUM(const string &traj_path,
                             vector<Vector8d> &traj_vec) {
    std::fstream stream(traj_path + "refined_traj_tum.txt", std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < traj_vec.size(); i++) {
        Vector8d matrix = traj_vec.at(i);
        // check the size of keyframeTimes
        stream << matrix(0, 0) << " " << matrix(1, 0) << " " << matrix(2, 0) << " "
               << matrix(3, 0) << " " << matrix(4, 0) << " " << matrix(5, 0) << " "
               << matrix(6, 0) << " " << matrix(7, 0) << std::endl;
    }
    stream.close();
}

template<typename T>
std::map<std::string, double> CloudMapEva::calculateError(
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

std::vector<double> CloudMapEva::computePointCloudDistance(
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

std::shared_ptr<PointCloud> CloudMapEva::renderDistanceOnPointCloud(
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
std::shared_ptr<open3d::geometry::PointCloud> CloudMapEva::ColorPointCloudByMME(
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

std::shared_ptr<open3d::geometry::PointCloud> CloudMapEva::ColorPointCloudByMME(
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


std::shared_ptr<open3d::geometry::PointCloud> CloudMapEva::ColorPointCloudByMME(
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

std::shared_ptr<Mesh> CloudMapEva::renderDistanceOnMesh(
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

shared_ptr<Mesh> CloudMapEva::createMeshFromPCD(
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

void CloudMapEva::getDiffRegResult(
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

void CloudMapEva::getDiffRegResult(
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

void CloudMapEva::getDiffRegResult(std::vector<Vector5d> &result,
                                   pipelines::registration::CorrespondenceSet &points_set,
                                   geometry::PointCloud &source,
                                   geometry::PointCloud &target,
                                   geometry::PointCloud &source_set,
                                   geometry::PointCloud &target_set) {
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

void CloudMapEva::calculateMetrics(pipelines::registration::RegistrationResult &registration_result) {
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
    for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
        double pr = 2 * est_gt_results.at(2)[i] * gt_est_results.at(2)[i];
        double p_plus_r = est_gt_results.at(2)[i] + gt_est_results.at(2)[i];
        f1_vec[i] = pr / p_plus_r;  // F1 Score
    }
    std::cout << "Chamfer Distance: " << cd_vec.transpose() << std::endl;
    std::cout << "F1 Score: " << f1_vec.transpose() << std::endl;
    std::cout << "MME: " << mme_est << " " << mme_gt << std::endl;
}

void CloudMapEva::saveResults() {
    // Save evaluation results to a file
    std::string result_file_path = param_.evaluation_map_pcd_path_ + "eval_result.txt";
    std::ofstream file_result(result_file_path.c_str(), std::ios::app);
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
    file_result << param_.evaluation_map_pcd_path_ + "map.pcd" << std::endl;
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
    file_result << std::fixed << setprecision(5) << "F1: " << f1_vec.transpose()
                << std::endl;
    if (eva_mme) {
        file_result << std::fixed << setprecision(5) << "MME: " << mme_est << " " << mme_gt << " "
                    << min_abs_entropy << " " << max_abs_entropy << std::endl;
    }
    file_result << "Time: " << t1 / 1000.0 << " " << (t2 - t1) / 1000.0 << " "
                << (t3 - t2) / 1000.0 << " " << (t4 - t3) / 1000.0 << " "
                << (t5 - t4) / 1000.0 << " " << (t7 - t5) / 1000.0 << std::endl;
    file_result.close();
    std::cout << "Results saved to " << result_file_path << std::endl;

    // rendering distance map
    map_3d_render_inlier = renderDistanceOnPointCloud(
            corresponding_cloud_gt, corresponding_cloud_est, param_.trunc_dist_[0]);
    map_3d_render_raw =
            renderDistanceOnPointCloud(gt_3d_, map_3d_, param_.trunc_dist_[0]);
    //    visualization::DrawGeometries({map_3d_render_inlier}, "mesh result");

    open3d::io::WritePointCloud(
            param_.evaluation_map_pcd_path_ + "raw_rendered_dis_map.pcd",
            *map_3d_render_raw);
    std::cout << "Save rendered inlier distance error map to "
              << param_.evaluation_map_pcd_path_ + "raw_rendered_dis_map.pcd"
              << std::endl;
    open3d::io::WritePointCloud(
            param_.evaluation_map_pcd_path_ + "inlier_rendered_dis_map.pcd",
            *map_3d_render_inlier);
    std::cout << "Save rendered raw distance error map to "
              << param_.evaluation_map_pcd_path_ + "inlier_rendered_dis_map.pcd"
              << std::endl;

    if (param_.evaluate_mme_) {
        open3d::io::WritePointCloud(
                param_.evaluation_map_pcd_path_ + "map_entropy.pcd",
                *map_3d_entropy);
        std::cout << "Save rendered entropy map to "
                  << param_.evaluation_map_pcd_path_ + "map_entropy.pcd"
                  << std::endl;
        if (param_.evaluate_gt_mme_) {
            open3d::io::WritePointCloud(
                    param_.evaluation_map_pcd_path_ + "gt_entropy.pcd",
                    *gt_3d_entropy);
            std::cout << "Save rendered entropy gt map to "
                      << param_.evaluation_map_pcd_path_ + "gt_entropy.pcd"
                      << std::endl;
            visualization::DrawGeometries({gt_3d_entropy}, "entropy result");
        }
        visualization::DrawGeometries({map_3d_entropy}, "entropy result");
    }
    //std::cout << "Results saved to " << result_file_path << std::endl;

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
        open3d::io::WriteTriangleMesh(param_.evaluation_map_pcd_path_ + "gt_mesh.ply",
                                      *gt_mesh);
        open3d::io::WriteTriangleMesh(param_.evaluation_map_pcd_path_ + "est_mesh.ply",
                                      *est_mesh);
        open3d::io::WriteTriangleMesh(
                param_.result_path_ + "correspondence_mesh.ply", *correspdence_mesh);
    }
    visualization::DrawGeometries({map_3d_render_raw}, "mesh result");
}

pipelines::registration::RegistrationResult CloudMapEva::performICPRegistration() {
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

double CloudMapEva::ComputeEntropy(const Eigen::Matrix3d &covariance) {
    double entropy = 0.5 * std::log(2 * M_PI * M_E * covariance.determinant());
    return entropy;
}

double
CloudMapEva::ComputeMeanMapEntropy(const std::shared_ptr<open3d::geometry::PointCloud> &pointcloud,
                                   std::vector<double> &entropies, double radius) {
    mean_entropy = 0.0;
    valid_points = 0;
    entropies = std::vector<double>(pointcloud->points_.size(), 0.0);
    valid_entropy_points.resize(pointcloud->points_.size(), false);
    StartProcessing(static_cast<int>(pointcloud->points_.size()));

    open3d::geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*pointcloud);

/*    for (size_t i = 0; i < pointcloud->points_.size(); ++i) {
        std::vector<int> indices;
        std::vector<double> distances;
        if (kdtree.SearchRadius(pointcloud->points_[i], radius, indices, distances) > 0) {
            // Remove the first index since it is the point itself
            indices.erase(indices.begin());
            distances.erase(distances.begin());
            if (indices.size() < 10) {
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
    }*/


    // Parallelize this loop with OpenMP
#pragma omp parallel for reduction(+:mean_entropy, valid_points)
    for (int i = 0; i < static_cast<int>(pointcloud->points_.size()); ++i) {
        std::vector<int> indices;
        std::vector<double> distances;
        if (kdtree.SearchRadius(pointcloud->points_[i], radius, indices, distances) > 0) {
            // Remove the first index since it is the point itself
            indices.erase(indices.begin());
            distances.erase(distances.begin());
            if (indices.size() < 15) {
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
    if (valid_points > 0) {
        mean_entropy /= valid_points;
    }
    std::cout << "MME Valid_points " << valid_points * 100.0 / total_points << "% " << valid_points << " "
              << total_points << std::endl;
    return mean_entropy;
}

void CloudMapEva::StartProcessing(int total) {
    start_time = std::chrono::steady_clock::now();
    total_points = total;
}

