#pragma once

#include <deque>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace super_lio_slam
{
  using RegistrationCloud = pcl::PointCloud<pcl::PointXYZI>;

  struct WindowEntry // 每个窗口记录位姿和点云
  {
    std::size_t index;                 // 索引
    double stamp;                      // 时间戳
    Eigen::Isometry3d odom_pose;       // 里程计位姿
    RegistrationCloud::ConstPtr cloud; // 点云
    double distance = 0.0;             // 距离值
  };

  struct SubmapLimits                 // 子图限制
  {
    int max_frames = 4;               // 最多帧数
    double max_duration = 6.0;        // 最长时间
    double max_travel = 6.0;          // 最长旅行距离
  };

  // Single-owner deque. Copies are small and share immutable single-frame clouds.
  class SubmapWindow
  {
  public:
    explicit SubmapWindow(SubmapLimits limits = {}) : limits_(limits) {}
    void append(WindowEntry entry);
    const std::deque<WindowEntry> &entries() const { return entries_; }
    RegistrationCloud::Ptr cloud(double voxel, std::size_t cap) const;
    bool sharesFrames(const SubmapWindow &other) const;

  private:
    SubmapLimits limits_;
    std::deque<WindowEntry> entries_; // 多个窗口入口
  };

  struct RegistrationSettings
  {
    double voxel = 0.5;
    int min_points = 100;
    int max_source = 10000;
    int max_target = 50000;
    double normal_radius = 1.0;
    double feature_radius = 2.5;
    double feature_ratio = 0.9;
    std::string feature_matching = "mutual_ratio";
    double hint_radius = 3.0;
    int min_pairs = 6;
    int max_pairs = 200;
    double min_baseline = 2.0;
    double noise_bound = 0.3;
    int rotation_iterations = 100;
    double clique_seconds = 0.05;
    bool exact_clique = false;
    double rotation_gnc_factor = 1.4;
    double rotation_cost_threshold = 1e-6;
    int threads = 2;
    double coarse_radius = 0.6;
    int coarse_min_inliers = 6;
    double coarse_min_ratio = 0.3;
    double coarse_max_rmse = 0.3;
    int max_iterations = 40;
    int covariance_neighbors = 20;
    double correspondence_distance = 2.0;
    double quality_radius = 1.0;
    double min_overlap = 0.8;         // strict lower bound
    double min_current_overlap = 0.8; // strict lower bound
    int min_inliers = 100;
    double max_mse = 0.1; // strict upper bound, m^2
    double min_information_ratio = 1e-5;
    double max_translation = 5.0;
    double max_rotation_deg = 20.0;
  };

  struct RegistrationResult
  {
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

  bool validRigidTransform(const Eigen::Matrix4d &transform);
  RegistrationResult coarseRegister(const RegistrationCloud::ConstPtr &source,
                                    const RegistrationCloud::ConstPtr &target,
                                    const RegistrationSettings &settings,
                                    const Eigen::Matrix4d *pose_hint = nullptr);
  RegistrationResult refineRegister(const RegistrationCloud::ConstPtr &source,
                                    const RegistrationCloud::ConstPtr &target,
                                    const RegistrationCloud::ConstPtr &current,
                                    const RegistrationSettings &settings,
                                    RegistrationResult coarse);
} // namespace super_lio_slam
