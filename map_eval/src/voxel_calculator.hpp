#ifndef VOXEL_LIO_INFORMATIONGAINCALCULATOR_H
#define VOXEL_LIO_INFORMATIONGAINCALCULATOR_H

#include <unordered_map>
#include <vector>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <deque>
#include <open3d/Open3D.h>

using PointType = pcl::PointXYZI;  // PointCloud type definition with intensity
using PCLPointCloud = pcl::PointCloud<PointType>;  // Define the PointCloud with PointType

// Custom hash function for Eigen::Vector3i, used as a key in unordered_map
struct VoxelHasher {
    std::size_t operator()(const Eigen::Vector3i &key) const {
        return std::hash<int>()(key[0]) ^ std::hash<int>()(key[1]) ^ std::hash<int>()(key[2]);
    }
};

// Voxel information structure to store mean, covariance, entropy, energy, and other properties
struct VoxelInfo {
    Eigen::Vector3d mu;  // Mean position of the points in the voxel
    Eigen::Matrix3d sigma;  // Covariance matrix for the voxel
    int num_points;  // Number of points inside the voxel
    double entropy;  // Entropy for the voxel, a measure of uncertainty
    double energy;  // Energy associated with the voxel, often related to the covariance

    int active;  // Voxel status: 0 - old, 1 - active, 2 - new
    double entropy_old;  // Previous entropy value to compare

    // Constructor to initialize voxel data
    VoxelInfo() : mu(Eigen::Vector3d::Zero()), sigma(Eigen::Matrix3d::Zero()), num_points(0), entropy(0),
                  entropy_old(0), energy(0), active(0) {}
};

// Voxel map structure to store VoxelInfo using Eigen::Vector3i as the key
using VoxelMap = std::unordered_map<Eigen::Vector3i, VoxelInfo, VoxelHasher>;


// VoxelCalculator class to handle voxel operations such as entropy calculation, Wasserstein distance, and map updates
class VoxelCalculator {
public:
    VoxelCalculator() = default;  // Default constructor

    // Constructor to initialize with a specific voxel size
    VoxelCalculator(double voxel_size);

    // Build voxel map from different types of point clouds (Open3D or PCL)
    void buildVoxelMap(const open3d::geometry::PointCloud &cloud);
    void buildVoxelMap(const PCLPointCloud::Ptr &cloud);

    // Update the voxel map with a new point cloud and pose transformation
    double updateVoxelMap(const PCLPointCloud::Ptr &cloud, const Eigen::Isometry3d &pose);
    void updateVoxelMap(const VoxelMap &map);
    double updateVoxelMap(const PCLPointCloud::Ptr &cloud);


    // Compute total entropy of the voxel map
    Eigen::Vector2d computeTotalEntropy();

    // Methods to compute Wasserstein distance, KLDivergence, and other metrics between voxel maps
    double computeWassersteinDistance(const VoxelMap &map1, const VoxelMap &map2);
    double computeWassersteinDistance2(const VoxelMap &map1, const VoxelMap &map2);
    double computeWassersteinDistance3(const VoxelMap &map1, const VoxelMap &map2);
    double computeWassersteinDistanceGaussian(const VoxelInfo &voxel1, const VoxelInfo &voxel2);

    double computeKLDivergenceGaussian(const VoxelInfo &voxel1, const VoxelInfo &voxel2);
    double computeKLDivergence(const VoxelMap &map1, const VoxelMap &map2);
    double computeKLDivergence2(const VoxelMap &map1, const VoxelMap &map2);

    double computeGaussianKernel(const Eigen::Vector3d &x, const Eigen::Vector3d &y, double sigma);
    double computeGaussianBhattacharyya(const VoxelInfo &voxel1, const VoxelInfo &voxel2, double sigma);

    double computeMMD(const VoxelMap &map1, const VoxelMap &map2, double sigma);
    double computeMMD2(const VoxelMap &map1, const VoxelMap &map2, double sigma);

    // Get the current voxel map
    const VoxelMap &getVoxelMap() const { return voxel_map_; }

    // Get the indices of neighboring voxels within a certain radius
    std::vector<Eigen::Vector3i> getNeighborIndices(const Eigen::Vector3i &index, int radius);

private:
    // Method to compute the entropy for a single voxel
    void computeVoxelEntropy(VoxelInfo &voxel);

    // Method to compute the voxel index based on a 3D point
    Eigen::Vector3i getVoxelIndex(const Eigen::Vector3d &point);

private:
    double voxel_size_;  // Size of each voxel

    VoxelMap voxel_map_;  // The main voxel map storing all voxel information
    VoxelMap voxel_map_tmp_;  // Temporary voxel map for certain operations

    // Variables to track the number of different types of voxels
    int active_num_ = 0;
    int new_area_num_ = 0;
    int old_area_num_ = 0;

    // Entropy and energy values for different regions of the map
    double total_entropy_;
    double total_energy_;
    double active_entropy_old_;
    double active_entropy_;
    double new_area_entropy_;
    double new_area_entropy_old_;
    double old_area_entropy_;
    double old_area_entropy_old_;

    // Store divergences and distances for specific voxels
    std::vector<std::pair<Eigen::Vector3i, double>> voxel_divergences;
    std::vector<std::pair<Eigen::Vector3i, double>> voxel_distances;
};

#endif //VOXEL_LIO_INFORMATIONGAINCALCULATOR_H
