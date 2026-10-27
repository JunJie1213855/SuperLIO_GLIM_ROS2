#pragma once

#include <deque>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace super_lio_slam {
using RegistrationCloud = pcl::PointCloud<pcl::PointXYZI>;

struct WindowEntry {
  std::size_t index;
  double stamp;
  Eigen::Isometry3d odom_pose;
  RegistrationCloud::ConstPtr cloud;
  double distance = 0.0;
};

struct SubmapLimits {
  int max_frames = 4;
  double max_duration = 6.0;
  double max_travel = 6.0;
};

// Single-owner deque. Copies are small and share immutable single-frame clouds.
class SubmapWindow {
 public:
  explicit SubmapWindow(SubmapLimits limits = {}) : limits_(limits) {}
  void append(WindowEntry entry);
  const std::deque<WindowEntry>& entries() const { return entries_; }
  RegistrationCloud::Ptr cloud(double voxel, std::size_t cap) const;
  bool sharesFrames(const SubmapWindow& other) const;
 private:
  SubmapLimits limits_;
  std::deque<WindowEntry> entries_;
};

struct RegistrationSettings {
  double voxel = 0.5;
  int min_points = 100;
  int max_source = 10000;
  int max_target = 50000;
  double normal_radius = 1.0;
  double feature_radius = 2.5;
  double feature_ratio = 0.9;
  double hint_radius = 3.0;
  int min_pairs = 6;
  int max_pairs = 200;
  double min_baseline = 2.0;
  double noise_bound = 0.3;
  int rotation_iterations = 100;
  double clique_seconds = 0.05;
  int threads = 2;
  double coarse_radius = 0.6;
  int coarse_min_inliers = 6;
  double coarse_min_ratio = 0.3;
  double coarse_max_rmse = 0.3;
  int max_iterations = 40;
  int covariance_neighbors = 20;
  double correspondence_distance = 2.0;
  double quality_radius = 1.0;
  double min_overlap = 0.35;
  double min_current_overlap = 0.35;
  int min_inliers = 100;
  double max_mse = 0.25;
  double min_information_ratio = 1e-5;
  double max_translation = 5.0;
  double max_rotation_deg = 20.0;
};

struct RegistrationResult {
  bool accepted = false;
  std::string reason;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity(); // historical <- current
  bool used_hint = false;
  std::size_t pairs = 0;
  std::size_t coarse_inliers = 0;
  double coarse_rmse = 0.0;
  std::size_t inliers = 0;
  double mse = 0.0;
  double overlap = 0.0;
  double current_overlap = 0.0;
  double coarse_ms = 0.0;
  double fine_ms = 0.0;
  double information_ratio = 0.0;
};

bool validRigidTransform(const Eigen::Matrix4d& transform);
RegistrationResult coarseRegister(const RegistrationCloud::ConstPtr& source,
                                  const RegistrationCloud::ConstPtr& target,
                                  const RegistrationSettings& settings,
                                  const Eigen::Matrix4d* pose_hint = nullptr);
RegistrationResult refineRegister(const RegistrationCloud::ConstPtr& source,
                                  const RegistrationCloud::ConstPtr& target,
                                  const RegistrationCloud::ConstPtr& current,
                                  const RegistrationSettings& settings,
                                  RegistrationResult coarse);
} // namespace super_lio_slam
