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
// include open3d
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

// KeyframeSelector class to decide if a given frame should be a keyframe based on distance and statistical methods
class KeyframeSelector {
public:
    KeyframeSelector(int window_size, double k, bool use_kl_divergence)
            : window_size_(window_size), k_(k), use_kl_divergence_(use_kl_divergence) {}

    // Method to determine if the current frame is a keyframe based on its distance and previous frame history
    bool isKeyframe(double distance) {
        distances_.push_back(distance);  // Store the current distance
        if (distances_.size() > window_size_) {
            distances_.pop_front();  // Keep the window size fixed
        }

        double median = getMedian(distances_);
        if (use_kl_divergence_) {
            std::vector<double> abs_diffs;
            for (double d : distances_) {
                abs_diffs.push_back(std::abs(d - median));
            }
            double mad = getMedian(abs_diffs);  // Mean Absolute Deviation
            threshold_ = median + k_ * mad;
            std::cout << "Threshold (KL Divergence): " << threshold_ << std::endl;
            return distance > threshold_;
        } else {
            threshold_ = median * (1 + k_);
            std::cout << "Threshold: " << threshold_ << std::endl;
            return distance > threshold_;
        }
    }

public:
    double threshold_ = 0.0;  // Threshold for keyframe decision

private:
    int window_size_;  // Size of the sliding window to store distances
    double k_;  // Sensitivity parameter for threshold calculation
    bool use_kl_divergence_;  // Flag to use KL divergence or median for threshold

    std::deque<double> distances_;  // Deque to store recent distances

    // Utility function to calculate the median of a deque of doubles
    double getMedian(const std::deque<double> &values) {
        std::deque<double> sorted_values = values;
        std::sort(sorted_values.begin(), sorted_values.end());
        int n = sorted_values.size();
        if (n % 2 == 0) {
            return (sorted_values[n / 2 - 1] + sorted_values[n / 2]) / 2.0;
        } else {
            return sorted_values[n / 2];
        }
    }

    // Utility function to calculate the median of a vector of doubles
    double getMedian(const std::vector<double> &values) {
        std::vector<double> sorted_values = values;
        std::sort(sorted_values.begin(), sorted_values.end());
        int n = sorted_values.size();
        if (n % 2 == 0) {
            return (sorted_values[n / 2 - 1] + sorted_values[n / 2]) / 2.0;
        } else {
            return sorted_values[n / 2];
        }
    }
};

// VoxelCalculator class to handle voxel operations such as entropy calculation, Wasserstein distance, and map updates
class VoxelCalculator {
public:
    VoxelCalculator() = default;  // Default constructor

    // Constructor to initialize with a specific voxel size
    VoxelCalculator(double voxel_size);

    // Method to compute information gain when a frame is added to the map
    double computeInformationGain(const PCLPointCloud::Ptr &frame, const Eigen::Isometry3d &pose);

    // Build voxel map from different types of point clouds (Open3D or PCL)
    void buildVoxelMap(const open3d::geometry::PointCloud &cloud);
    void buildVoxelMap(const PCLPointCloud::Ptr &cloud);
    void buildVoxelMap(const PCLPointCloud::Ptr &cloud, double alpha);

    // Update the voxel map with a new point cloud and pose transformation
    double updateVoxelMap(const PCLPointCloud::Ptr &cloud, const Eigen::Isometry3d &pose);
    void updateVoxelMap(const VoxelMap &map);
    double updateVoxelMap(const PCLPointCloud::Ptr &cloud);

    // Remove points from the voxel map and reset their associated voxels
    void removePointCloudAndResetVoxels(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    // Compute total entropy of the voxel map
    Eigen::Vector2d computeTotalEntropy();

    // Compute Renyi entropy for each voxel
    void computeVoxelRenyiEntropyPower(VoxelInfo &voxel, double alpha);

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
