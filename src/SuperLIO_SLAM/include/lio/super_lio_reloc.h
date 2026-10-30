#pragma once

#include "super_lio.h"
#include "lio/RelocationRegistration.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include <std_msgs/msg/string.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/msg/marker.hpp>

namespace LI2Sup
{
  class SuperLIOReLoc final : public SuperLIO
  {
  public:
    SuperLIOReLoc() = default;
    ~SuperLIOReLoc() override;
    void init() override;
    void process() override;

  private:
    enum class Stage
    {
      Bootstrap,
      Local,
      World,
      Failed
    };
    bool initializeLocal();
    void rememberLocalPose();
    void enqueueRegistration();
    void registrationWorker();
    bool commitRegistration();
    void receivePrior(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
    void setStatus(const std::string &status);
    void limitScan();
    void Output() override;
    void publishLocalMapPreview(bool clear = false);
  private:
    Stage stage_ = Stage::Bootstrap;                 // 状态
    std::string status_;
    std::string debug_snapshot_dir_;
    OctVoxMapType::Ptr local_ivox_, reference_ivox_; // 当前局部，参考
    BASIC::CloudPtr reference_cloud_;
    pcl::KdTreeFLANN<BASIC::PointType> reference_tree_;
    struct LocalPose
    {
      double stamp;
      BASIC::SE3 pose;
    };
    // Pose-only timestamp lookup for /initialpose, never a registration cloud window.
    std::deque<LocalPose> prior_pose_history_;
    super_lio_slam::RegistrationSettings settings_;
    std::deque<IMUData> initialization_imu_;
    std::size_t imu_samples_ = 50;
    int registration_min_frames_ = 5, min_plane_points_ = 30;
    double init_duration_ = .3, max_gyro_ = .15, max_gyro_std_ = .02, max_accel_std_ = .15;
    double registration_interval_ = 1.0, crop_radius_ = 80.0, gravity_angle_ = 10.0;
    double position_variance_ = .04, rotation_variance_ = .0012184697;
    double last_scan_time_ = -1, last_task_time_ = -1;
    bool tracking_good_ = false, have_prior_ = true;
    BASIC::SE3 prior_pose_, prior_local_pose_;
    std::uint64_t generation_ = 1;
    std::size_t local_frame_count_ = 0;
    std::mutex registration_mutex_;
    std::condition_variable registration_cv_;
    std::optional<super_lio_slam::RelocationSnapshot> pending_task_;
    std::optional<super_lio_slam::RelocationResult> pending_result_;
    std::atomic<bool> stopping_{false};
    std::thread registration_thread_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_map_preview_pub_;
    bool visualize_local_map_ = true;
    double local_map_preview_interval_ = 0.5;
    double last_local_map_preview_time_ = -1;
    std::size_t local_map_preview_max_points_ = 50000;
  };
} // namespace LI2Sup
