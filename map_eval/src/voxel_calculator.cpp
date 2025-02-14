#include "voxel_calculator.hpp"

VoxelCalculator::VoxelCalculator(double voxel_size) {
    voxel_size_ = voxel_size;
}

std::vector<Eigen::Vector3i> VoxelCalculator::getNeighborIndices(const Eigen::Vector3i &index, int radius) {
    std::vector<Eigen::Vector3i> neighbors;
    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dz = -radius; dz <= radius; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;  // Skip the center voxel
                neighbors.emplace_back(index + Eigen::Vector3i(dx, dy, dz));
            }
        }
    }
    return neighbors;
}

void VoxelCalculator::buildVoxelMap(const open3d::geometry::PointCloud &cloud) {
    voxel_map_.clear();
    total_entropy_ = 0.0;
    total_energy_ = 0.0;
    for (const auto &point : cloud.points_) {
        Eigen::Vector3i voxel_index = getVoxelIndex(point);
        auto it = voxel_map_.find(voxel_index);
        if (it == voxel_map_.end()) {
            VoxelInfo voxel_info;
            voxel_info.num_points = 1;
            voxel_info.mu = point;
            voxel_info.sigma = Eigen::Matrix3d::Zero();
            voxel_info.energy = voxel_info.sigma.trace();
            voxel_info.active = 1;  // All voxels are initially active
            voxel_map_.emplace(voxel_index, voxel_info);
        } else {
            VoxelInfo &voxel_info = it->second;
            voxel_info.num_points++;
            Eigen::Vector3d delta = point - voxel_info.mu;
            voxel_info.mu += delta / voxel_info.num_points;
            voxel_info.sigma += delta * (point - voxel_info.mu).transpose();
            voxel_info.energy = voxel_info.sigma.trace();
        }
    }
    for (auto &kv : voxel_map_) {
        VoxelInfo &voxel_info = kv.second;
        if (voxel_info.num_points > 10) {
            voxel_info.sigma /= (voxel_info.num_points - 1);
            computeVoxelEntropy(voxel_info);
            voxel_info.entropy_old = voxel_info.entropy;
            total_entropy_ += voxel_info.entropy;
            total_energy_ += voxel_info.energy;
        }
    }
    std::cout << "Build voxel map: " << total_entropy_ << " " << voxel_map_.size() << std::endl;
}

void VoxelCalculator::buildVoxelMap(const PCLPointCloud::Ptr &cloud) {
    voxel_map_.clear();
    total_entropy_ = 0.0;
    for (const auto &point : cloud->points) {
        Eigen::Vector3d point_world = point.getVector3fMap().cast<double>();
        Eigen::Vector3i voxel_index = getVoxelIndex(point_world);
        auto it = voxel_map_.find(voxel_index);
        if (it == voxel_map_.end()) {
            VoxelInfo voxel_info;
            voxel_info.num_points = 1;
            voxel_info.mu = point_world;
            voxel_info.sigma = Eigen::Matrix3d::Zero();
            voxel_info.energy = voxel_info.sigma.trace();
            voxel_map_.emplace(voxel_index, voxel_info);
        } else {
            VoxelInfo &voxel_info = it->second;
            voxel_info.num_points++;
            Eigen::Vector3d delta = point_world - voxel_info.mu;
            voxel_info.mu += delta / voxel_info.num_points;
            voxel_info.sigma += delta * (point_world - voxel_info.mu).transpose();
            voxel_info.energy = voxel_info.sigma.trace();
        }
    }
    for (auto &kv : voxel_map_) {
        VoxelInfo &voxel_info = kv.second;
        if (voxel_info.num_points > 1) {
            voxel_info.sigma /= (voxel_info.num_points - 1);
            computeVoxelEntropy(voxel_info);
            voxel_info.entropy_old = voxel_info.entropy;
            total_entropy_ += kv.second.entropy;
            total_energy_ += kv.second.energy;
        } else {
            voxel_info.sigma = Eigen::Matrix3d::Identity() * 1e-6;
            voxel_info.energy = voxel_info.sigma.trace();
        }
    }
    std::cout << "Build voxel map: " << total_entropy_ << " " << voxel_map_.size() << std::endl;
}

void VoxelCalculator::computeVoxelEntropy(VoxelInfo &voxel) {
    if (voxel.num_points < 2) {
        voxel.entropy = 0;
        voxel.energy = 0;
    } else {
        voxel.sigma /= (voxel.num_points - 1);
        double det = voxel.sigma.determinant();
        if (det <= 0) {
            voxel.entropy = 0;
            voxel.energy = 0;
        } else {
            constexpr double PI = 3.141592653589793238463;
            voxel.entropy = 0.5 * std::log(std::pow(2 * PI * std::exp(1), 3) * det);
            voxel.energy = voxel.sigma.trace();
        }
    }
}

double VoxelCalculator::computeWassersteinDistanceGaussian(const VoxelInfo &voxel1, const VoxelInfo &voxel2) {
    Eigen::Vector3d mu1 = voxel1.mu;
    Eigen::Vector3d mu2 = voxel2.mu;
    Eigen::Matrix3d sigma1 = Eigen::Matrix3d::Identity();
    if (voxel1.num_points > 1) {
        sigma1 = voxel1.sigma / (voxel1.num_points - 1);
        sigma1 = (sigma1 + sigma1.transpose()) / 2;  // Ensure symmetry
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(sigma1);
        sigma1 = eigensolver.eigenvectors() * eigensolver.eigenvalues().cwiseMax(1e-6).asDiagonal() *
                 eigensolver.eigenvectors().transpose();
    }
    Eigen::Matrix3d sigma2 = Eigen::Matrix3d::Identity();
    if (voxel2.num_points > 1) {
        sigma2 = voxel2.sigma / (voxel2.num_points - 1);
        sigma2 = (sigma2 + sigma2.transpose()) / 2;  // Ensure symmetry
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(sigma2);
        sigma2 = eigensolver.eigenvectors() * eigensolver.eigenvalues().cwiseMax(1e-6).asDiagonal() *
                 eigensolver.eigenvectors().transpose();
    }
    Eigen::Vector3d mu_diff = mu1 - mu2;
    Eigen::Matrix3d sigma_sum = sigma1 + sigma2;
    Eigen::Matrix3d sigma_sqrt = sigma1.llt().matrixL() * sigma2 * sigma1.llt().matrixL().transpose();
    sigma_sqrt = sigma_sqrt.llt().matrixL();
    double distance = mu_diff.dot(mu_diff) + sigma_sum.trace() - 2 * sigma_sqrt.trace();
    return std::sqrt(std::max(0.0, distance));  // Ensure non-negative distance
}

void VoxelCalculator::updateVoxelMap(const VoxelMap &gt_map) {
    int old_area_num = 0, active_num = 0, new_area_num = 0;
    // First, mark all existing voxels as new
    for (auto &kv : voxel_map_) {
        kv.second.active = 2;  // new area
    }
    // Compare with GT map and update labels
    for (const auto &gt_kv : gt_map) {
        const Eigen::Vector3i &index = gt_kv.first;
        auto it = voxel_map_.find(index);
        if (it != voxel_map_.end()) {
            it->second.active = 1;  // active
            active_num++;
        } else {
            // GT voxel doesn't exist in EST map
            VoxelInfo voxel_info;
            voxel_info.active = 0;  // old area
            voxel_map_[index] = voxel_info;
            old_area_num++;
        }
    }
    // Count remaining new areas
    for (const auto &kv : voxel_map_) {
        if (kv.second.active == 2) {
            new_area_num++;
        }
    }
    std::cout << "Update voxel map: " << voxel_map_.size() << std::endl;
    std::cout << "Update active/old/new voxel num: " << active_num << " " << old_area_num << " " << new_area_num
              << std::endl;
}


double VoxelCalculator::updateVoxelMap(const PCLPointCloud::Ptr &cloud) {
    voxel_map_tmp_ = voxel_map_;

    // Reset all voxels to old_area
    for (auto &kv : voxel_map_) {
        kv.second.active = 0;
    }

    for (const auto &point : cloud->points) {
        Eigen::Vector3d point_world = point.getVector3fMap().cast<double>();
        Eigen::Vector3i voxel_index = getVoxelIndex(point_world);
        auto it = voxel_map_.find(voxel_index);
        if (it == voxel_map_.end()) {
            // do not find the existing voxel, add new voxel
            VoxelInfo voxel_info;
            voxel_info.mu = point_world;
            voxel_info.num_points = 1;
            computeVoxelEntropy(voxel_info);
            voxel_map_.emplace(voxel_index, voxel_info);
            // set entropy_old for new voxel
            voxel_map_[voxel_index].entropy_old = voxel_info.entropy;
            // set entropy_old for new voxel
            // voxel_map_[voxel_index].entropy_old = 0.0;
            // mark as new area
            voxel_map_[voxel_index].active = 2;
        } else {
            // find the existing voxel
            VoxelInfo &voxel = it->second;
            Eigen::Vector3d old_mu = voxel.mu;

            // save old entropy
            auto it2 = voxel_map_tmp_.find(voxel_index);
            if (it2 != voxel_map_tmp_.end()) {
                voxel.entropy_old = it2->second.entropy;
            }
            // update gaussian params
            voxel.mu = (voxel.mu * voxel.num_points + point_world) / (voxel.num_points + 1);
            voxel.sigma = (voxel.num_points - 1.0) / voxel.num_points * voxel.sigma
                          + (point_world - old_mu) * (point_world - voxel.mu).transpose();
            voxel.num_points++;
            computeVoxelEntropy(voxel);
            // mark as active voxel if it's not new area
            if (voxel.active != 2) {
                voxel.active = 1;
            }
        }
    }
    // mark voxels that are not updated as old area
    old_area_num_ = active_num_ = new_area_num_ = 0;
    for (auto &kv : voxel_map_) {
        if (kv.second.active == 0) {
            //kv.second.entropy_old = kv.second.entropy;
            old_area_num_++;
        } else if (kv.second.active == 1) {
            active_num_++;
        } else if (kv.second.active == 2) {
            new_area_num_++;
        }
    }
    std::cout << "update voxel map: " << voxel_map_.size() << " " << voxel_map_tmp_.size() << std::endl;
    std::cout << "update active/old/new voxel num: " << active_num_ << " " << old_area_num_ << " " << new_area_num_
              << std::endl;
    return total_entropy_;
}


Eigen::Vector3i VoxelCalculator::getVoxelIndex(const Eigen::Vector3d &point) {
    return Eigen::Vector3i(std::floor(point.x() / voxel_size_),
                           std::floor(point.y() / voxel_size_),
                           std::floor(point.z() / voxel_size_));
}
