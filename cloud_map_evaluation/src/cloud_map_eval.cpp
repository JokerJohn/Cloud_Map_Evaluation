#include "cloud_map_eval.h"

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
      line_reader >> pose(0, 0) >> pose(1, 0) >> pose(2, 0) >> pose(3, 0) >> pose(4, 0) >> pose(5, 0) >> pose(6, 0) >> pose(7, 0);
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

int CloudMapEva::createGridMap() {
  //  std::cout << gt_3d_->GetMaxBound() << std::endl;
  //  std::cout << gt_3d_->GetMinBound() << std::endl;
  //  PointCloud gt_slice(*gt_3d_);
  //  if (param_.voxel_dim_ == 2) {
  //    for (int i = 0; i < gt_slice.points_.size(); i++) {
  //      gt_slice.points_[i][2] = 0;
  //    }
  //  }
  //  voxel_grid_gt_ = open3d::geometry::VoxelGrid::CreateFromPointCloud(
  //      gt_slice, param_.voxel_size_);
  //  open3d::visualization::DrawGeometries({voxel_grid_gt_}, "gt
  //  voxelization");
}

int CloudMapEva::crop(shared_ptr<PointCloud> &reference_points,
                      shared_ptr<PointCloud> &target_points) {
  PointCloud ref_slice(*reference_points);
  //  if (param_.voxel_dim_ == 2) {
  //    for (int i = 0; i < ref_slice.points_.size(); i++) {
  //      ref_slice.points_[i][2] = 0;
  //    }
  //  }
  //  std::shared_ptr<open3d::geometry::VoxelGrid> voxel_grid =
  //      open3d::geometry::VoxelGrid::CreateFromPointCloud(ref_slice,
  //                                                        param_.outlier_dist_);
  //  PointCloud target_slice(*target_points);
  //  if (param_.voxel_dim_ == 2) {
  //    for (int i = 0; i < target_slice.points_.size(); i++) {
  //      target_slice.points_[i][2] = 0;
  //    }
  //  }

  //  auto crop_vec = voxel_grid->CheckIfIncluded(target_slice.points_);
  //  std::vector<size_t> indices;
  //  for (int index = 0; index < crop_vec.size(); index++) {
  //    if (crop_vec[index]) {
  //      indices.push_back(index);
  //    }
  //  }
  //  target_points = target_points->SelectByIndex(indices);
  //  LOG_IF(INFO, param_.verbosity_ > 2)
  //      << "after crop map has points: " << target_points->points_.size();
  // open3d::visualization::DrawGeometries({target_points}, "map after crop");
}

template<typename T>
int CloudMapEva::calculateRMSE(std::vector<T> &result_vec) {
  double sum = std::accumulate(result_vec.begin(), result_vec.end(), 0.0);
  double num = result_vec.size();
  double mean = sum / num;
  double accum = 0.0;
  std::for_each(result_vec.begin(), result_vec.end(),
                [&](const double x) { accum += (x - mean) * (x - mean); });
  double rmse = sqrt(accum / num);
  auto [min, max] =
      std::minmax_element(std::begin(result_vec), std::end(result_vec));
  double dis_range = (*max) - (*min);
  //  LOG_IF(INFO, param_.verbosity_ > 1)
  //      << "\n [RMSE results]:"
  //      << "\n number of points pairs: " << num << "\n sun of distance: " <<
  //      sum
  //      << "\n mean of distance: " << mean << "\n rmse: " << rmse
  //      << "\n min distance: " << *min << "\n max distance: " << *max
  //      << "\n dis range: " << dis_range;
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

double CloudMapEva::evaluateChamferDistance(
    std::shared_ptr<PointCloud> &reference_points,
    std::shared_ptr<PointCloud> &target_points) {
  //////////////////////// point distance 1
  std::vector<double> eval_dis1 =
      computePointCloudDistance(reference_points, target_points);
  std::map<std::string, double> result1 = calculateError(eval_dis1);

  //////////////////////// point distance 2
  std::vector<double> eval_dis2 =
      computePointCloudDistance(target_points, reference_points);
  std::map<std::string, double> result2 = calculateError(eval_dis2);

  //////////////////////// compute overall distance
  double chamfer_dist_l2_root =
      std::sqrt(0.5 * (result1["rmse"] * result1["rmse"] + result2["rmse"] * result2["rmse"]));
  double chamfer_dist_l1 = 0.5 * (result1["mean"] + result2["mean"]);
  return chamfer_dist_l1;
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

  Vector5i number_vec = Vector5i ::Zero();
  Vector5d fitness_vec = Vector5d ::Zero();
  Vector5d mean_vec = Vector5d ::Zero();
  Vector5d rmse_vec = Vector5d ::Zero();
  Vector5d sigma_vec = Vector5d ::Zero();

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

  Vector5i number_vec = Vector5i ::Zero();
  Vector5d fitness_vec = Vector5d ::Zero();
  Vector5d mean_vec = Vector5d ::Zero();
  Vector5d rmse_vec = Vector5d ::Zero();
  Vector5d sigma_vec = Vector5d ::Zero();

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

int CloudMapEva::process() {
  // load traj, but still exists some bugs, do not use it
  bool optimize_traj = false;
  vector<Vector8d> pose_vec;
  if (optimize_traj) {
    loadTrajTUM(param_.evaluation_map_pcd_path_, pose_vec);
    std::cout << "load traj poses: " << pose_vec.size() << std::endl;
  }

  TicToc tic_toc;
  io::ReadPointCloudOption io_params("auto", true, true, true);
  open3d::io::ReadPointCloudFromPCD(param_.evaluation_map_pcd_path_ + "map.pcd",
                                    *map_3d_, io_params);
  open3d::io::ReadPointCloudFromPCD(param_.map_gt_path_, *gt_3d_, io_params);

  // we can also use other api to read point cloud map
  //  map_3d_ = open3d::io::CreatePointCloudFromFile(
  //      param_. + "map.pcd");
  //  if (map_3d_->IsEmpty()) {
  //    std::cout << "Couldn't read file pcd for " << param_.name_ <<
  //    std::endl; return (-1);
  //  }
  //  gt_3d_ = open3d::io::CreatePointCloudFromFile(param_.map_gt_path_);
  //  if (gt_3d_->IsEmpty()) {
  //    std::cout << "Couldn't read file pcd for " << param_.name_ <<
  //    std::endl; return (-1);
  //  }

  if (map_3d_->IsEmpty() || gt_3d_->IsEmpty()) {
    std::cout << "empty point cloud !!!!!" << std::endl;
    return -1;
  }
  int source_num = map_3d_->points_.size();
  int target_num = gt_3d_->points_.size();

  // if we want to downsample point cloud, do this
  //  map_3d_ = map_3d_->VoxelDownSample(0.01);
  //  gt_3d_ = gt_3d_->VoxelDownSample(0.01);

  double t1 = tic_toc.toc();
  std::cout << "1. load point size: " << map_3d_->points_.size() << ", "
            << gt_3d_->points_.size() << " " << t1 / 1000.0 << " [s]"
            << std::endl;

  // if we have point cloud with normals, we do not to caculate normals again
  // since it cost so much time
  if (!map_3d_->HasNormals()) {
    std::cout << "write normal cloud map to "
              << param_.result_path_ + "est_map_normal.pcd" << std::endl;
    map_3d_->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 100));
    open3d::io::WritePointCloud(param_.result_path_ + "est_map_normal.pcd",
                                *map_3d_);
  }
  if (!gt_3d_->HasNormals()) {
    std::cout << "write normal gt cloud map to "
              << param_.result_path_ + "gt_normal.pcd" << std::endl;
    gt_3d_->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 100));
    open3d::io::WritePointCloud(param_.result_path_ + "gt_normal.pcd", *gt_3d_);
  }
  double t2 = tic_toc.toc();
  std::cout << "2. estimate normal: " << (t2 - t1) / 1000.0 << " [s]"
            << std::endl;

  // if we want to caculate mesh
  bool eva_mesh = false;
  shared_ptr<Mesh> gt_mesh(new Mesh()), est_mesh(new Mesh());
  shared_ptr<Mesh> gt_mesh_filtered(new Mesh()), est_mesh_filtered(new Mesh());
  if (eva_mesh) {
    // from possion
    gt_mesh = createMeshFromPCD(gt_3d_, 0.6, 10);
    est_mesh = createMeshFromPCD(map_3d_, 0.6, 10);
  }
  double t3 = tic_toc.toc();
  std::cout << "3. create mesh: " << (t3 - t2) / 1000.0 << " [s]" << std::endl;

  *map_3d_ = map_3d_->Transform(param_.initial_matrix_);
  // open3d::visualization::DrawGeometries({map_3d_, gt_3d_}, "init align");

  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  pipelines::registration::RegistrationResult registration_result;
  auto criteria = pipelines::registration::ICPConvergenceCriteria();
  switch (param_.evaluation_method_) {
    case 0:// point-to-point icp
      registration_result = pipelines::registration::RegistrationICP(
          *map_3d_, *gt_3d_, param_.icp_max_distance_,
          Eigen::Matrix4d::Identity(),
          pipelines::registration::TransformationEstimationPointToPoint(),
          criteria);
      break;
    case 1:// Point-to-plane
      registration_result = pipelines::registration::RegistrationICP(
          *map_3d_, *gt_3d_, param_.icp_max_distance_,
          Eigen::Matrix4d::Identity(),
          pipelines::registration::TransformationEstimationPointToPlane(),
          criteria);
      break;
    case 2:
      registration_result =
          pipelines::registration::RegistrationGeneralizedICP(
              *map_3d_, *gt_3d_, param_.icp_max_distance_,
              Eigen::Matrix4d::Identity(),
              pipelines::registration::
                  TransformationEstimationForGeneralizedICP(),
              criteria);
      break;
    default:
      std::cout << " evaluation error type!!!!! " << std::endl;
      break;
  }

  double t4 = tic_toc.toc();
  trans = registration_result.transformation_;
  *map_3d_ = map_3d_->Transform(trans);
  //        open3d::visualization::DrawGeometries({map_3d_},
  //                                              "Registration result");

  std::cout << "4. aligned cloud : " << (t4 - t3) / 1000.0 << " [s]"
            << std::endl;

  // update new poses, just perform a rigid transformation to algin traj
  if (optimize_traj) {
    std::shared_ptr<PointCloud> traj_cloud(new PointCloud());
    for (int i = 0; i < pose_vec.size(); ++i) {
      Vector8d &matrix = pose_vec.at(i);
      Matrix4d pose = Matrix4d::Identity();
      pose.block<3, 3>(0, 0) =
          Quaterniond(matrix(7, 0), matrix(4, 0), matrix(5, 0), matrix(6, 0))
              .toRotationMatrix();
      pose.block<3, 1>(0, 3) =
          Vector3d(matrix(1, 0), matrix(2, 0), matrix(3, 0));

      Quaterniond q(pose.block<3, 3>(0, 0).matrix());
      matrix(1, 0) = pose.block<3, 1>(0, 3)[0];
      matrix(2, 0) = pose.block<3, 1>(0, 3)[1];
      matrix(3, 0) = pose.block<3, 1>(0, 3)[2];
      matrix(4, 0) = q.x();
      matrix(5, 0) = q.y();
      matrix(6, 0) = q.z();
      matrix(7, 0) = q.w();
      traj_cloud->points_.push_back(pose.block<3, 1>(0, 3));
    }
    saveTrajTUM(param_.result_path_, pose_vec);
    open3d::io::WritePointCloud(
        param_.result_path_ + param_.name_ + "_traj_pcd_optimized.pcd",
        *traj_cloud);
  }
  double t5 = tic_toc.toc();

  std::cout << "5. update pose : " << (t5 - t4) / 1000.0 << " [s]"
            << std::endl;

  // visualization::DrawGeometries({gt_3d_, traj_cloud},
  //                                       "Registration result");

  // we caculate the result of 1cm 2cm 5cm 10cm and  20cm
  // inclding mean value/rmse/overlap/chamfer distance/f1 score

  std::cout << "aligned transformation: \n"
            << trans << std::endl;
  std::cout << "icp overlap ratio: " << registration_result.fitness_
            << std::endl;
  std::cout << "icp corressopndence rmse: "
            << registration_result.inlier_rmse_ << std::endl;
  std::cout << "icp corressopndence size: "
            << registration_result.correspondence_set_.size() << std::endl;

  if (registration_result.inlier_rmse_ > 0.2 || registration_result.fitness_ < 0.6) {
    std::cerr
        << "poor registeration result, please check your initial matrix!!!!"
        << std::endl;
    return -1;
  }

  // get corressponding cloud
  int corres_size = registration_result.correspondence_set_.size();
  shared_ptr<PointCloud> corresponding_cloud_est(new PointCloud());
  shared_ptr<PointCloud> corresponding_cloud_gt(new PointCloud());
  vector<double> est_gt_dis(corres_size, 0.0);

  std::vector<Vector5d> est_gt_sults;
  getDiffRegResult(est_gt_sults, registration_result.correspondence_set_, *map_3d_, *gt_3d_);

  // TODO:  caculate cov using severial correspondences
  //  MatrixXd corres_set_source, corres_set_target;
  //  getDiffRegResult(est_gt_sults, registration_result.correspondence_set_,
  //                   *map_3d_, *gt_3d_, corres_set_source, corres_set_target);

  double t6 = tic_toc.toc();

  std::cout << "\n6. caculate est-gt metrics : " << (t6 - t5) / 1000.0
            << " [s]" << std::endl;
  std::cout << std::fixed << setprecision(5)
            << "est-gt rmse: " << est_gt_sults.at(1).transpose() << std::endl;
  std::cout << std::fixed << setprecision(5)
            << "est_gt mean: " << est_gt_sults.at(0).transpose() << std::endl;
  std::cout << std::fixed << setprecision(5)
            << "standard deviation: " << est_gt_sults.at(3).transpose()
            << std::endl;
  std::cout << std::fixed << setprecision(5)
            << "fitness: " << est_gt_sults.at(2).transpose() << std::endl;

  auto result_gt_est = pipelines::registration::EvaluateRegistration(
      *gt_3d_, *map_3d_, param_.icp_max_distance_);
  std::vector<Vector5d> gt_est_sults;
  getDiffRegResult(gt_est_sults, result_gt_est.correspondence_set_, *gt_3d_,
                   *map_3d_);

  if (0) {
    std::cout << std::fixed << setprecision(5)
              << "20/10/5/2/1 gt_est mean: " << gt_est_sults.at(0).transpose()
              << std::endl;
    std::cout << std::fixed << setprecision(5)
              << "inlier rmse: " << gt_est_sults.at(1).transpose()
              << std::endl;
    std::cout << std::fixed << setprecision(5)
              << "inlier fitness: " << gt_est_sults.at(2).transpose()
              << std::endl;
    std::cout << std::fixed << setprecision(5)
              << "inlier standard deviation: "
              << gt_est_sults.at(3).transpose() << "\n"
              << std::endl;
  }

  Vector5d cd_vec = Vector5d ::Zero();// pow   d1/s1 +d2/s2
  Vector5d f1_vec = Vector5d ::Zero();
  cd_vec = est_gt_sults.at(1) + gt_est_sults.at(1);

  for (int i = 0; i < param_.trunc_dist_.size(); ++i) {
    double pr = 2 * est_gt_sults.at(2)[i] * gt_est_sults.at(2)[i];
    double p_plus_r = est_gt_sults.at(2)[i] + gt_est_sults.at(2)[i];
    f1_vec[i] = pr / p_plus_r;
  }

  std::cout << std::fixed << setprecision(5) << "CD: " << cd_vec.transpose()
            << std::endl;
  std::cout << std::fixed << setprecision(5) << "F1: " << f1_vec.transpose()
            << std::endl;
  double t7 = tic_toc.toc();
  std::cout << "7. caculate gt-est metrics : " << (t7 - t6) / 1000.0 << " [s]"
            << std::endl;

  // create corresspondence mesh
  shared_ptr<Mesh> correspdence_mesh(new Mesh());
  if (0) {
    corresponding_cloud_est->EstimateNormals(
        geometry::KDTreeSearchParamHybrid(1.0, 50));
    correspdence_mesh = createMeshFromPCD(corresponding_cloud_est, 0.6, 11);
  }
  // visualization::DrawGeometries({correspdence_mesh},
  // "correspdence_mesh");

  // debug
  open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
  // rendering distance map
  shared_ptr<PointCloud> map_3d_render_inlier = renderDistanceOnPointCloud(
      corresponding_cloud_gt, corresponding_cloud_est, param_.trunc_dist_[4]);
  shared_ptr<PointCloud> map_3d_render_raw =
      renderDistanceOnPointCloud(gt_3d_, map_3d_, param_.trunc_dist_[0]);
  // visualization::DrawGeometries({map_3d_render_raw}, "mesh result");

  // TODO : render mesh by distance error
  // std::cout << "render corresspondence mesh begin" << std::endl;
  //    shared_ptr<Mesh> est_mesh_map_render =
  //        renderDistanceOnMesh(corresponding_cloud_gt, correspdence_mesh,
  //                             corresponding_cloud_est, 0.1);
  //    visualization::DrawGeometries({est_mesh_map_render}, "mesh result");
  //    std::cout << "render corresspondence mesh end" << std::endl;

  if (param_.save_immediate_result_) {
    // output result to a file
    std::string result_file_path = param_.result_path_ + "eval_result.txt";
    std::ofstream file_result(result_file_path.c_str(), std::ios::app);
    file_result << param_.name_ << std::endl;
    file_result << param_.map_gt_path_ << std::endl;
    file_result << param_.evaluation_map_pcd_path_ + "map.pcd" << std::endl;
    file_result << std::fixed << std::setprecision(15)
                << est_gt_sults.at(1).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15)
                << est_gt_sults.at(0).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15)
                << est_gt_sults.at(3).transpose() << std::endl;
    file_result << std::fixed << std::setprecision(15)
                << est_gt_sults.at(2).transpose() << std::endl;
    file_result << std::fixed << setprecision(5) << cd_vec.transpose()
                << std::endl;
    file_result << std::fixed << setprecision(5) << f1_vec.transpose()
                << std::endl;
    file_result << t1 / 1000.0 << " " << (t2 - t1) / 1000.0 << " "
                << (t3 - t2) / 1000.0 << " " << (t4 - t3) / 1000.0 << " "
                << (t5 - t4) / 1000.0 << " " << (t6 - t5) / 1000.0 << " "
                << (t7 - t6) / 1000.0 << std::endl;
    file_result.close();

    open3d::io::WritePointCloud(
        param_.result_path_ + "eval_render_dis_raw_map.pcd",
        *map_3d_render_raw);
    open3d::io::WritePointCloud(
        param_.result_path_ + "eval_render_dis_inlier_map.pcd",
        *map_3d_render_inlier);
    visualization::DrawGeometries({map_3d_render_raw}, "mesh result");

    // ToDO: can not save mesh file successfully
    if (eva_mesh) {
      // gt mesh only save once
      open3d::io::WriteTriangleMesh(param_.result_path_ + "gt_mesh.ply",
                                    *gt_mesh);
      open3d::io::WriteTriangleMesh(param_.result_path_ + "est_mesh.ply",
                                    *est_mesh);
      //        open3d::io::WriteTriangleMesh(
      //            param_.result_path_ + "est_render_dis_mesh.ply",
      //            *est_mesh_map_render);
      open3d::io::WriteTriangleMesh(
          param_.result_path_ + "correspdence_mesh.ply", *correspdence_mesh);
    }
  }
  return (0);
}
