#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <optional>
#include <thread>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "STDesc.h"
#include "loop/SubmapRegistration.hpp"
#include "super_lio_slam/msg/key_frame.hpp"

namespace super_lio_slam {

class LoopBackendNode final : public rclcpp::Node {
 public:
  LoopBackendNode();
  ~LoopBackendNode() override;

 private:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  using CloudPtr = Cloud::Ptr;

  struct Frame {
    uint32_t input_id = 0;
    builtin_interfaces::msg::Time stamp;
    gtsam::Pose3 odom_pose;
    gtsam::Pose3 map_pose;
    CloudPtr cloud;
  };

  struct RegistrationObservation {
    gtsam::Pose3 historical_from_current;
    double fitness = 0.0;
    double overlap = 0.0;
  };

  struct RegistrationCandidate {
    std::size_t historical_index = 0;
    SubmapWindow target_window;
    std::vector<Frame> local_frames;
    gtsam::Pose3 local_anchor;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> proposal{
        Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()};
    bool have_pose_hint = false;
    std::optional<Eigen::Matrix4d> odom_hint;
  };
  struct RegistrationTask {
    std::size_t index = 0;
    CloudPtr current_cloud;
    SubmapWindow source_window;
    std::vector<RegistrationCandidate> candidates;
    std::chrono::steady_clock::time_point enqueued;
  };
  struct MapSnapshot {
    std::vector<Frame> frames;
    builtin_interfaces::msg::Time stamp;
    std::chrono::steady_clock::time_point enqueued;
  };
  void registrationWorker();
  void mapWorker();
  void enqueueRegistration(RegistrationTask task);
  bool loopTimeEligible(const builtin_interfaces::msg::Time& current,
                        std::size_t historical) const; // graph_mutex_ held
  bool loopCandidateEligible(const builtin_interfaces::msg::Time& current,
                             const gtsam::Pose3& raw_pose,
                             std::size_t historical) const; // graph_mutex_ held
  void commitLoop(std::size_t index, std::size_t historical,
                  const RegistrationObservation& observation);
  void requestMap(const builtin_interfaces::msg::Time& stamp);

  void onKeyFrame(const super_lio_slam::msg::KeyFrame::SharedPtr msg);
  CloudPtr convertAndFilter(const sensor_msgs::msg::PointCloud2 &message) const;
  CloudPtr downsample(const CloudPtr &cloud, double voxel_size,
                      std::size_t max_points) const;
  std::unique_ptr<RegistrationObservation> verifyWithGicp(
      const CloudPtr &current_cloud, const CloudPtr &target, std::size_t current_index,
      std::size_t historical_index,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &proposal,
      const Eigen::Matrix4d *odom_hint) const;

  void addGraphNode(std::size_t index, const gtsam::Pose3 &raw_pose);
  void publishCurrent(std::size_t index, bool loop_accepted);
  void publishMapToWorld();
  void publishPath(const builtin_interfaces::msg::Time &stamp);
  void publishMap(const MapSnapshot& snapshot);
  void publishLoopMarkers(const builtin_interfaces::msg::Time &stamp);
  void publishStdTriangles(std::size_t index,
                           const std::vector<STDesc> &descriptors);

  static gtsam::Pose3 poseFromMsg(const geometry_msgs::msg::Pose &pose);
  static geometry_msgs::msg::Pose poseToMsg(const gtsam::Pose3 &pose);

  std::unique_ptr<RegistrationObservation> verifySubmaps(
      const SubmapWindow& source, const SubmapWindow& target, const CloudPtr& current_cloud,
      std::size_t historical_index, const Eigen::Matrix4d* hint) const;
  std::string registration_mode_;
  std::string candidate_mode_;
  double candidate_radius_m_ = 15.0;
  SubmapLimits submap_limits_;
  SubmapWindow source_window_;
  RegistrationSettings registration_settings_;
  int registration_top_k_ = 3;
  double registration_budget_ms_ = 500.0;
  STDescManager std_manager_;
  std::vector<Frame> frames_;
  std::vector<std::pair<std::size_t, std::size_t>> loops_;
  gtsam::ISAM2 isam_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
  gtsam::noiseModel::Base::shared_ptr loop_noise_;
  gtsam::Pose3 map_from_world_;
  mutable std::mutex map_tf_mutex_;
  bool have_map_transform_ = false;

  rclcpp::Subscription<super_lio_slam::msg::KeyFrame>::SharedPtr keyframe_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr std_marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr tf_callback_group_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  double keyframe_cloud_voxel_size_ = 0.25;
  int keyframe_cloud_max_points_ = 30000;
  int gicp_submap_half_width_ = 5;
  int loop_min_interval_ = 20;
  double loop_min_time_separation_s_ = 30.0;
  int loop_diagnostic_stride_ = 1;
  int map_publish_stride_ = 30;
  int map_max_keyframes_ = 2000;
  int map_max_points_per_keyframe_ = 1000;
  double map_voxel_size_ = 0.5;
  int path_max_poses_ = 10000;
  int std_visualization_max_triangles_ = 100;
  double tf_publish_rate_hz_ = 20.0;
  std::size_t last_loop_index_ = 0;
  bool have_loop_ = false;
  bool have_input_id_ = false;
  uint32_t last_input_id_ = 0;
  std::atomic<bool> backend_faulted_{false};
  // Graph and published pose state are short-held; no registration/map work under this mutex.
  std::mutex graph_mutex_;
  std::mutex registration_mutex_;
  std::condition_variable registration_cv_;
  std::deque<RegistrationTask> registration_tasks_;
  std::size_t registration_queue_capacity_ = 2;
  std::size_t registration_tasks_dropped_ = 0;
  std::mutex map_mutex_;
  std::condition_variable map_cv_;
  std::optional<MapSnapshot> pending_map_;
  std::thread registration_thread_;
  std::thread map_thread_;
  std::atomic<bool> stopping_{false};
};

}  // namespace super_lio_slam
