#include "loop/LoopBackendNode.hpp"
#include "loop/OptimizedMapWriter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>

namespace super_lio_slam
{
  namespace
  {

    // Reset immediately after each highlighted field to keep numeric diagnostics plain.
    std::string decisionText(bool accepted, const std::string &text)
    {
      return std::string(accepted ? "\033[32m" : "\033[31m") + text + "\033[0m";
    }

    gtsam::Vector6 poseSigmas(double rotation, double translation)
    {
      gtsam::Vector6 sigma;
      sigma << rotation, rotation, rotation, translation, translation, translation;
      return sigma;
    }

    gtsam::ISAM2Params onlineGraphParameters()
    {
      gtsam::ISAM2Params parameters;
      parameters.relinearizeThreshold = 0.01;
      parameters.relinearizeSkip = 1;
      return parameters;
    }

  } // namespace

  LoopBackendNode::LoopBackendNode()
      : rclcpp::Node("loop_backend_node"), isam_(onlineGraphParameters())
  {
    // STD uses its own contiguous index for descriptors and plane clouds. It is
    // deliberately independent of the incoming ID so reconnects are harmless.
    std_manager_.current_frame_id_ = 0;
    read_parameters(*this, std_manager_.config_setting_);

    // 闭环检测候选方法选择：std 和 distance
    candidate_mode_ = declare_parameter<std::string>("loop_candidate_mode", "std");
    // 两个选项，两个都不是直接丢异常
    if (candidate_mode_ != "distance" && candidate_mode_ != "std")
      throw std::invalid_argument("loop_candidate_mode must be distance or std");
    // 闭环整体方法，默认为 std + teaserpp + gicp 方法
    registration_mode_ = declare_parameter<std::string>("loop_registration_mode", "std_teaser_gicp");
    if (registration_mode_ != "std_gicp" && registration_mode_ != "std_teaser_gicp")
      throw std::invalid_argument("loop_registration_mode must be std_gicp or std_teaser_gicp");
#ifndef SUPERLIO_WITH_TEASER
    if (registration_mode_ == "std_teaser_gicp")
      throw std::invalid_argument("std_teaser_gicp requires a build with SUPERLIO_WITH_TEASER=ON");
#endif

    // 辅助函数：返回大于 0 的值
    auto positive = [this](const char *name, double value)
    {
      const double result = declare_parameter<double>(name, value);
      if (!std::isfinite(result) || result <= 0)
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
      return result;
    };
    // 辅助函数：返回在区间内的值
    auto bounded = [this](const char *name, int value, int low, int high)
    {
      const int result = declare_parameter<int>(name, value);
      if (result < low || result > high)
        throw std::invalid_argument(std::string(name) + " outside supported range");
      return result;
    };

    // 基础参数设置
    candidate_radius_m_ = positive("loop_candidate_radius_m", 15.0);
    submap_limits_.max_frames = bounded("registration_submap_max_keyframes", 4, 1, 20);
    submap_limits_.max_duration = positive("registration_submap_max_duration_s", 6.0);
    submap_limits_.max_travel = positive("registration_submap_max_travel_distance_m", 6.0);
    source_window_ = SubmapWindow(submap_limits_);
    registration_top_k_ = bounded("std_registration_top_k", 3, 1, 10);
    registration_budget_ms_ = positive("registration_soft_budget_ms", 500.0);
    registration_queue_capacity_ = bounded("registration_queue_capacity", 2, 1, 100);

    // 设置
    auto &settings = registration_settings_;
    settings.voxel = positive("registration_voxel_size_m", 0.5);
    settings.min_points = bounded("registration_min_points", 100, 30, 100000);
    settings.max_source = bounded("registration_max_source_points", 10000, 100, 100000);
    settings.max_target = bounded("registration_max_target_points", 50000, 100, 200000);
    settings.normal_radius = positive("fpfh_normal_radius_m", 1.0);
    settings.feature_radius = positive("fpfh_feature_radius_m", 2.5);
    if (settings.feature_radius <= settings.normal_radius)
      throw std::invalid_argument("FPFH feature radius must exceed normal radius");
    settings.feature_ratio = positive("fpfh_distance_ratio_threshold", 0.9);
    settings.hint_radius = positive("fpfh_pose_hint_radius_m", 3.0);
    if (settings.feature_ratio > 1)
      throw std::invalid_argument("FPFH distance ratio must be <= 1");
    settings.min_pairs = bounded("correspondence_min_unique", 6, 3, 500);
    settings.max_pairs = bounded("correspondence_max_unique", 200, 3, 500);
    if (settings.min_pairs > settings.max_pairs)
      throw std::invalid_argument("minimum pairs exceeds maximum");
    settings.min_baseline = positive("correspondence_min_baseline_m", 2.0);
    if (declare_parameter<bool>("teaser_estimate_scaling", false))
      throw std::invalid_argument("LiDAR registration requires teaser_estimate_scaling=false");
    settings.noise_bound = positive("teaser_noise_bound_m", 0.3);
    settings.rotation_iterations = bounded("teaser_rotation_max_iterations", 100, 1, 1000);
    settings.clique_seconds = positive("teaser_max_clique_time_limit_s", 0.05);
    settings.threads = bounded("teaser_max_clique_num_threads", 2, 1, 16);
    settings.coarse_radius = positive("coarse_residual_threshold_m", 0.6);
    settings.coarse_min_inliers = bounded("coarse_min_inliers", 6, 3, 500);
    settings.coarse_min_ratio = positive("coarse_min_inlier_ratio", 0.3);
    settings.coarse_max_rmse = positive("coarse_max_inlier_rmse_m", 0.3);
    settings.max_iterations = bounded("gicp_max_iterations", 40, 1, 200);
    settings.covariance_neighbors = bounded("gicp_correspondence_randomness", 20, 4, 100);
    settings.correspondence_distance = positive("gicp_max_correspondence_distance_m", 2.0);
    settings.quality_radius = positive("quality_nn_radius_m", 1.0);
    settings.min_overlap = positive("quality_min_source_overlap", 0.8);
    settings.min_current_overlap = positive("quality_min_current_frame_overlap", 0.8);
    settings.min_inliers = bounded("quality_min_inlier_count", 100, 3, 100000);
    settings.max_mse = positive("quality_max_inlier_mse_m2", 0.1);
    settings.min_information_ratio = positive("quality_min_information_ratio", 1e-5);
    settings.max_translation = positive("quality_max_refinement_translation_m", 5.0);
    settings.max_rotation_deg = positive("quality_max_refinement_rotation_deg", 20.0);
    if (settings.coarse_min_ratio > 1 || settings.min_overlap >= 1 || settings.min_current_overlap >= 1)
      throw std::invalid_argument("coarse inlier ratio must be <= 1; strict GICP overlap bounds must be < 1");

    // 普通参数声明
    keyframe_cloud_voxel_size_ = declare_parameter<double>("keyframe_cloud_voxel_size", 0.25);
    keyframe_cloud_max_points_ = declare_parameter<int>("keyframe_cloud_max_points", 30000);

    gicp_submap_half_width_ = bounded("gicp_submap_half_width", 5, 0, 100);

    // 回环的部分判决条件
    loop_min_interval_ = declare_parameter<int>("loop_min_interval", 20);
    loop_min_time_separation_s_ = declare_parameter<double>("loop_min_time_separation_s", 30.0);
    if (!std::isfinite(loop_min_time_separation_s_) || loop_min_time_separation_s_ < 0.0)
      throw std::invalid_argument("loop_min_time_separation_s must be finite and nonnegative");
    loop_diagnostic_stride_ = declare_parameter<int>("loop_diagnostic_stride", 1);
    map_publish_stride_ = declare_parameter<int>("map_publish_stride", 30);
    map_max_keyframes_ = declare_parameter<int>("map_max_keyframes", 2000);
    map_max_points_per_keyframe_ =
        declare_parameter<int>("map_max_points_per_keyframe", 1000);
    map_voxel_size_ = declare_parameter<double>("map_voxel_size", 0.5);
    save_map_path_ = std::filesystem::absolute(
                         declare_parameter<std::string>("save_map_path", "map/optimized_map.pcd"))
                         .string();
    save_map_voxel_size_ = declare_parameter<double>("save_map_voxel_size", 0.2);
    save_map_on_shutdown_ = declare_parameter<bool>("save_map_on_shutdown", false);
    if (!std::isfinite(save_map_voxel_size_) || save_map_voxel_size_ < 0)
      throw std::invalid_argument("save_map_voxel_size must be finite and nonnegative");
    path_max_poses_ = declare_parameter<int>("path_max_poses", 10000);
    std_visualization_max_triangles_ =
        declare_parameter<int>("std_visualization_max_triangles", 100);
    tf_publish_rate_hz_ = declare_parameter<double>("tf_publish_rate_hz", 20.0);

    // gtsam 回环优化中的先验信息
    const double prior_trans = declare_parameter<double>("graph_prior_translation_sigma", 0.001);
    const double prior_rot = declare_parameter<double>("graph_prior_rotation_sigma", 0.001);
    const double odom_trans = declare_parameter<double>("graph_odom_translation_sigma", 0.10);
    const double odom_rot = declare_parameter<double>("graph_odom_rotation_sigma", 0.03);
    const double loop_trans = declare_parameter<double>("graph_loop_translation_sigma", 0.25);
    const double loop_rot = declare_parameter<double>("graph_loop_rotation_sigma", 0.05);
    const double huber_k = declare_parameter<double>("graph_loop_huber_k", 1.5);

    // iSAM2 PGO的噪声协方差设置
    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        poseSigmas(std::max(1e-6, prior_rot), std::max(1e-6, prior_trans)));
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        poseSigmas(std::max(1e-6, odom_rot), std::max(1e-6, odom_trans)));
    loop_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(std::max(0.01, huber_k)),
        gtsam::noiseModel::Diagonal::Sigmas(
            poseSigmas(std::max(1e-6, loop_rot), std::max(1e-6, loop_trans))));

    // 关键帧挑选
    keyframe_cloud_max_points_ = std::max(100, keyframe_cloud_max_points_);
    loop_min_interval_ = std::max(0, loop_min_interval_);
    loop_diagnostic_stride_ = std::max(0, loop_diagnostic_stride_);
    map_publish_stride_ = std::max(1, map_publish_stride_);
    map_max_points_per_keyframe_ = std::max(1, map_max_points_per_keyframe_);
    path_max_poses_ = std::max(1, path_max_poses_);
    std_visualization_max_triangles_ =
        std::clamp(std_visualization_max_triangles_, 0, 1000);
    if (!std::isfinite(tf_publish_rate_hz_))
      tf_publish_rate_hz_ = 20.0;

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    // 发布
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lio_slam/odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/lio_slam/path", output_qos);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lio_slam/map", output_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lio_slam/loop_markers", output_qos);
    std_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lio_slam/std_triangles", output_qos);
    // tf 广播器和定时器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tf_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    tf_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 /
                                                   std::clamp(tf_publish_rate_hz_, 1.0, 100.0))),
        [this]()
        {
          publishMapToWorld();
        },
        tf_callback_group_);
    // Track data time independently of /clock, including IMU-rate pose displays.
    rclcpp::SubscriptionOptions time_options;
    time_options.callback_group = tf_callback_group_;
    const auto time_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();

    // 订阅 superlio 的位姿和关键帧
    frontend_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/odom", time_qos,
        std::bind(&LoopBackendNode::onFrontendOdometry, this, std::placeholders::_1),
        time_options);
    frontend_imu_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/imu/odom", time_qos,
        std::bind(&LoopBackendNode::onFrontendOdometry, this, std::placeholders::_1),
        time_options);
    // 关键帧订阅：重要
    keyframe_sub_ = create_subscription<super_lio_slam::msg::KeyFrame>(
        "/lio_slam/keyframe", rclcpp::QoS(rclcpp::KeepLast(200)).reliable(),
        std::bind(&LoopBackendNode::onKeyFrame, this, std::placeholders::_1));
    // Same mutually-exclusive group as keyframe ingest: drain registration, then
    // capture a consistent final graph. TF callbacks can continue independently.
    save_map_service_ = create_service<std_srvs::srv::Trigger>(
        "/lio_slam/save_optimized_map",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          waitForRegistration();
          response->success = saveOptimizedMap(response->message);
        });
    // 配准线程
    registration_thread_ = std::thread(&LoopBackendNode::registrationWorker, this);
    try
    {
      // 地图线程
      map_thread_ = std::thread(&LoopBackendNode::mapWorker, this);
    }
    catch (...)
    {
      stopping_ = true;
      registration_cv_.notify_all();
      registration_thread_.join();
      throw;
    }
    RCLCPP_INFO(get_logger(), "Loop backend ready: candidates=%s radius=%.1f m, %s + iSAM2, async registration queue=%zu",
                candidate_mode_.c_str(), candidate_radius_m_, registration_mode_.c_str(), registration_queue_capacity_);
  }

  gtsam::Pose3 LoopBackendNode::poseFromMsg(const geometry_msgs::msg::Pose &pose)
  {
    const auto &p = pose.position;
    const auto &r = pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(r.x) || !std::isfinite(r.y) || !std::isfinite(r.z) ||
        !std::isfinite(r.w))
    {
      throw std::invalid_argument("non-finite keyframe pose");
    }
    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    if (q.norm() < 1e-6)
    {
      throw std::invalid_argument("zero keyframe quaternion");
    }
    q.normalize();
    return gtsam::Pose3(gtsam::Rot3(q.w(), q.x(), q.y(), q.z()),
                        gtsam::Point3(p.x, p.y, p.z));
  }

  geometry_msgs::msg::Pose LoopBackendNode::poseToMsg(const gtsam::Pose3 &pose)
  {
    geometry_msgs::msg::Pose output;
    const auto p = pose.translation();
    const auto q = pose.rotation().toQuaternion();
    output.position.x = p.x();
    output.position.y = p.y();
    output.position.z = p.z();
    output.orientation.x = q.x();
    output.orientation.y = q.y();
    output.orientation.z = q.z();
    output.orientation.w = q.w();
    return output;
  }

  LoopBackendNode::CloudPtr LoopBackendNode::downsample(
      const CloudPtr &cloud, double voxel_size, std::size_t max_points) const
  {
    CloudPtr filtered(new Cloud);
    if (!cloud || cloud->empty())
      return filtered;
    if (voxel_size > 0.01)
    {
      pcl::VoxelGrid<pcl::PointXYZI> voxel;
      voxel.setLeafSize(static_cast<float>(voxel_size),
                        static_cast<float>(voxel_size),
                        static_cast<float>(voxel_size));
      voxel.setInputCloud(cloud);
      voxel.filter(*filtered);
    }
    else
    {
      *filtered = *cloud;
    }
    if (max_points > 0 && filtered->size() > max_points)
    {
      CloudPtr capped(new Cloud);
      capped->reserve(max_points);
      const std::size_t stride =
          (filtered->size() + max_points - 1) / max_points;
      for (std::size_t i = 0; i < filtered->size(); i += stride)
      {
        capped->push_back((*filtered)[i]);
      }
      return capped;
    }
    return filtered;
  }

  LoopBackendNode::CloudPtr LoopBackendNode::convertAndFilter(
      const sensor_msgs::msg::PointCloud2 &message) const
  {
    CloudPtr raw(new Cloud);
    pcl::fromROSMsg(message, *raw);
    CloudPtr finite(new Cloud);
    finite->reserve(raw->size());
    for (auto p : *raw)
    {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      {
        if (!std::isfinite(p.intensity))
          p.intensity = 0.0f;
        finite->push_back(p);
      }
    }
    return downsample(finite, keyframe_cloud_voxel_size_,
                      static_cast<std::size_t>(keyframe_cloud_max_points_));
  }

  std::unique_ptr<LoopBackendNode::RegistrationObservation> LoopBackendNode::verifyWithGicp(
      const CloudPtr &current_cloud, const CloudPtr &target, std::size_t current_index,
      std::size_t historical_index,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &proposal,
      const Eigen::Matrix4d *odom_hint) const
  {
    const auto &settings = registration_settings_;
    const auto source = downsample(current_cloud, settings.voxel, settings.max_source);
    RegistrationResult initial;
    initial.accepted = true;
    initial.transform.block<3, 3>(0, 0) = proposal.second;
    initial.transform.block<3, 1>(0, 3) = proposal.first;
    auto result = refineRegister(source, target, source, settings, initial);
    const char *selected_hint = "proposal";
    // Repeated structures can give STD a plausible but locally incorrect hint.
    // Try the continuous odometry hint as well; both attempts use identical gates.
    if (odom_hint && (*odom_hint - initial.transform).norm() > 1e-6)
    {
      initial.transform = *odom_hint;
      const auto alternative = refineRegister(source, target, source, settings, initial);
      RCLCPP_DEBUG(get_logger(), "GICP odometry attempt %zu -> %zu: stage=%s mse=%.4f overlap=%.3f",
                   historical_index, current_index, decisionText(alternative.accepted, alternative.reason).c_str(), alternative.mse, alternative.overlap);
      const auto cost = [&settings](const RegistrationResult &candidate)
      {
        // Penalize unmatched source points instead of rewarding a tiny good subset.
        return candidate.overlap * candidate.mse +
               (1.0 - candidate.overlap) * settings.quality_radius * settings.quality_radius;
      };
      if (alternative.accepted && (!result.accepted || cost(alternative) < cost(result)))
      {
        result = alternative;
        selected_hint = "odom";
      }
    }
    RCLCPP_INFO(get_logger(),
                "GICP %zu -> %zu %s: initial=%s mse=%.4f m2 overlap=%.3f "
                "current_overlap=%.3f inliers=%zu info_ratio=%.6f stage=%s",
                historical_index, current_index,
                decisionText(result.accepted, result.accepted ? "accepted" : "rejected").c_str(),
                selected_hint, result.mse, result.overlap,
                result.current_overlap, result.inliers, result.information_ratio,
                decisionText(result.accepted, result.reason).c_str());
    if (!result.accepted)
      return nullptr;
    auto observation = std::make_unique<RegistrationObservation>();
    observation->historical_from_current = gtsam::Pose3(result.transform);
    observation->fitness = result.mse;
    observation->overlap = result.overlap;
    return observation;
  }

  std::unique_ptr<LoopBackendNode::RegistrationObservation> LoopBackendNode::verifySubmaps(
      const SubmapWindow &source_window, const SubmapWindow &target_window, const CloudPtr &current_cloud,
      std::size_t historical_index, const Eigen::Matrix4d *hint) const
  {
    // 源子图锚定当前帧 C，目标子图锚定历史帧 H，各自内部用原始 LIO 相对位姿拼接。
    // 因此配准直接返回 T_H_C，无需在入图前额外乘锚点位姿。
    if (source_window.sharesFrames(target_window))
    {
      RCLCPP_DEBUG(get_logger(), "Submap %zu %s: stage=%s", historical_index,
                   decisionText(false, "rejected").c_str(),
                   decisionText(false, "shared_window_members").c_str());
      return nullptr;
    }
    const auto &settings = registration_settings_;
    const auto source = source_window.cloud(settings.voxel, settings.max_source);
    const auto target = target_window.cloud(settings.voxel, settings.max_target);
    const auto current = downsample(current_cloud, settings.voxel, settings.max_source);

    // Teaser++ 粗配准
    auto result = coarseRegister(source, target, settings, hint);

    // 如果可以接受，就进行精配准，采用 GICP
    if (result.accepted)
      result = refineRegister(source, target, current, settings, result);
    // 提示可能来自错误的重复结构；有提示的粗/精配准失败后，以相同验收标准无提示重试。
    // Hints reduce repeated-structure ambiguity but never gate candidate admission.
    // Retry without the hint if either registration stage rejects the hinted pairs.
    if (!result.accepted && hint)
    {
      RCLCPP_DEBUG(get_logger(), "Submap %zu hinted registration %s; retry unguided stage=%s",
                   historical_index, decisionText(false, "rejected").c_str(),
                   decisionText(false, result.reason).c_str());
      const double previous_ms = result.coarse_ms + result.fine_ms;
      result = coarseRegister(source, target, settings);
      if (result.accepted)
        result = refineRegister(source, target, current, settings, result);
      result.coarse_ms += previous_ms;
    }
    RCLCPP_INFO(get_logger(),
                "Registration %zu -> %zu %s: coarse_inliers=%zu/%zu ratio=%.3f (min %.3f) "
                "coarse_rmse=%s m (max %.3f) gicp=%s stage=%s",
                historical_index, source_window.entries().back().index,
                decisionText(result.accepted, result.accepted ? "accepted" : "rejected").c_str(),
                result.coarse_inliers, result.pairs,
                result.pairs ? static_cast<double>(result.coarse_inliers) / result.pairs : 0.0,
                settings.coarse_min_ratio,
                result.coarse_inliers ? std::to_string(result.coarse_rmse).c_str() : "n/a",
                settings.coarse_max_rmse, result.fine_ms > 0.0 ? "executed" : "skipped",
                decisionText(result.accepted, result.reason).c_str());
    if (result.fine_ms > 0.0)
    {
      RCLCPP_INFO(get_logger(),
                  "GICP %zu -> %zu: mse=%.4f m2 overlap=%.3f current_overlap=%.3f "
                  "inliers=%zu info_ratio=%.6f",
                  historical_index, source_window.entries().back().index,
                  result.mse, result.overlap, result.current_overlap, result.inliers, result.information_ratio);
    }
    if (!result.accepted)
      return nullptr;
    // 如果精配准可以接受，那么将其进行传出
    auto observation = std::make_unique<RegistrationObservation>();
    observation->historical_from_current = gtsam::Pose3(result.transform);
    observation->fitness = result.mse;
    observation->overlap = result.overlap;
    return observation;
  }

  void LoopBackendNode::addGraphNode(
      std::size_t index, const gtsam::Pose3 &raw_pose)
  {
    // 图变量 X_i = T_map_Ii。首帧先验固定全局自由度，后续边来自未经 PGO 改写的 LIO 相对运动。
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    if (index == 0)
    {
      factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, raw_pose, prior_noise_));
      initial.insert(0, raw_pose);
    }
    else
    {
      // Z_(i-1,i) = T_world_I(i-1)^-1 * T_world_Ii；新节点初值接在上一优化位姿后面。
      const auto raw_delta = frames_[index - 1].odom_pose.between(raw_pose);
      factors.add(gtsam::BetweenFactor<gtsam::Pose3>(index - 1, index,
                                                     raw_delta, odom_noise_));
      initial.insert(index, frames_[index - 1].map_pose.compose(raw_delta));
    }
    // 更新因子图
    isam_.update(factors, initial);
    frames_[index].map_pose = isam_.calculateEstimate<gtsam::Pose3>(index);
    {
      std::lock_guard<std::mutex> lock(map_tf_mutex_);
      // 同一关键帧的两套位姿相消，得到 T_map_world = T_map_I * T_world_I^-1。
      map_from_world_ = frames_[index].map_pose.compose(raw_pose.inverse());
      have_map_transform_ = true;
      map_tf_dirty_ = true;
    }
  }

  void LoopBackendNode::onKeyFrame(
      const super_lio_slam::msg::KeyFrame::SharedPtr msg)
  {
    // 阅读主线：预处理/候选召回 -> 关键帧入库及里程计因子 -> 异步几何验证 -> commitLoop。
    // 本回调独占 STD 状态；重计算交给配准线程，关键帧和里程计边不能因配准排队而丢失。
    if (backend_faulted_)
      return;
    if (have_input_id_ && msg->id <= last_input_id_)
    {
      RCLCPP_WARN(get_logger(), "Ignoring duplicate or out-of-order keyframe %u",
                  msg->id);
      return;
    }
    const auto processing_begin = std::chrono::steady_clock::now();
    gtsam::Pose3 raw_pose;
    CloudPtr cloud;
    try
    {
      raw_pose = poseFromMsg(msg->odom_pose);
      cloud = convertAndFilter(msg->cloud); // 转换为 pcl 格式，并且降采样
    }
    catch (const std::exception &error)
    {
      RCLCPP_WARN(get_logger(), "Ignoring keyframe %u: %s", msg->id,
                  error.what());
      return;
    }
    if (cloud->size() < 10) // 去除点数少的点云
    {
      RCLCPP_WARN(get_logger(), "Ignoring sparse keyframe %u (%zu points)",
                  msg->id, cloud->size());
      return;
    }
    if (have_input_id_ && msg->id != last_input_id_ + 1)
    {
      RCLCPP_WARN(get_logger(), "Keyframe ID gap: %u followed by %u; using dense backend indices",
                  last_input_id_, msg->id);
    }

    const double filter_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - processing_begin)
                                 .count();
    std::size_t index;
    bool enough_time_since_loop;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      index = frames_.size();
      enough_time_since_loop = !have_loop_ || index - last_loop_index_ >=
                                                  static_cast<std::size_t>(loop_min_interval_);
    }

    // 仅回环 std_teaser_gicp 维护有限多帧窗口；不要与重定位的固定 S 系累积地图混淆。
    auto pending_window = source_window_;
    if (registration_mode_ == "std_teaser_gicp")
    {
      try
      {
        pending_window.append({index, rclcpp::Time(msg->header.stamp).seconds(),
                               Eigen::Isometry3d(raw_pose.matrix()), cloud});
      }
      catch (const std::exception &error)
      {
        RCLCPP_WARN(get_logger(), "Ignoring keyframe %u: %s", msg->id, error.what());
        return;
      }
    }

    // 注册一个配准任务，让配准线程执行
    RegistrationTask task;
    task.index = index;
    task.current_cloud = cloud;
    task.source_window = pending_window;

    // STD 描述子，用于匹配
    std::vector<STDesc> descriptors;
    const auto plane_count = std_manager_.plane_cloud_vec_.size();
    const auto corner_count = std_manager_.corner_cloud_vec_.size();
    bool adding_descriptors = false;
    double descriptor_ms = 0.0;
    const auto std_begin = std::chrono::steady_clock::now();
    // candidate_mode 只决定“找谁”；registration_mode 决定“如何验证”，二者独立配置。
    if (candidate_mode_ == "distance") // 采用距离决定候选帧
    {
      if (enough_time_since_loop)
      {
        // Rank in the continuous raw odometry frame; never mix optimized and raw poses.
        std::vector<std::pair<double, std::size_t>> nearest;
        std::lock_guard<std::mutex> lock(graph_mutex_);
        for (std::size_t h = 0; h < index; ++h)
        {
          if (!loopCandidateEligible(msg->header.stamp, raw_pose, h)) // 检测是否在两帧的时间是否过短
            continue;
          const double distance = (frames_[h].odom_pose.translation() - raw_pose.translation()).norm();
          if (distance <= candidate_radius_m_) // 检测两帧距离满足阈值
            nearest.emplace_back(distance, h);
        }
        std::sort(nearest.begin(), nearest.end()); // 从最小的开始配准
        std::vector<std::size_t> selected;
        for (const auto &[distance, h] : nearest)
        {
          if (selected.size() >= static_cast<std::size_t>(registration_top_k_)) // 有足够多的帧就跳出循环
            break;
          if (std::any_of(selected.begin(), selected.end(), [h](std::size_t previous)
                          { return h > previous ? h - previous <= 2 : previous - h <= 2; }))
            continue;
          RegistrationCandidate match;
          match.historical_index = h;
          const auto proposal = frames_[h].odom_pose.between(raw_pose);
          match.proposal = {proposal.translation(), proposal.rotation().matrix()};
          match.have_pose_hint = true;
          if (registration_mode_ == "std_teaser_gicp")
          {
            match.target_window = SubmapWindow(submap_limits_); // 添加目标子图
            const std::size_t first = h + 1 > static_cast<std::size_t>(submap_limits_.max_frames)
                                          ? h + 1 - submap_limits_.max_frames
                                          : 0;
            for (std::size_t j = first; j <= h; ++j) // 添加多帧子图
            {
              const auto &frame = frames_[j];
              match.target_window.append({j, rclcpp::Time(frame.stamp).seconds(),
                                          Eigen::Isometry3d(frame.odom_pose.matrix()), frame.cloud});
            }
            if (pending_window.sharesFrames(match.target_window))
              continue;
          }
          else
          {
            const auto half = static_cast<std::size_t>(std::max(0, gicp_submap_half_width_));
            const auto first = h > half ? h - half : 0;
            const auto last = std::min(index - 1, h + half);
            match.local_anchor = frames_[h].odom_pose;
            match.local_frames.assign(frames_.begin() + first, frames_.begin() + last + 1);
          }
          selected.push_back(h);
          const double gap = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(frames_[h].stamp)).seconds();
          RCLCPP_DEBUG(get_logger(), "Distance candidate %zu -> %zu: distance=%.2f m time_gap=%.3f s",
                       h, index, distance, gap);
          task.candidates.push_back(std::move(match));
        }
      }
    }
    else // STD 描述子候选
    {
      try
      {
        // Only this callback owns STD state; workers consume immutable snapshots.
        // 当前帧先生成描述子并查询历史库，搜索完成后才插入，避免把自身召回成回环。
        std_manager_.GenerateSTDescs(cloud, descriptors); // 生成 STD 描述子
        descriptor_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - std_begin)
                            .count();
        if (enough_time_since_loop && !descriptors.empty())
        {
          if (registration_mode_ == "std_teaser_gicp")
          {
            // STD 在此只负责地点召回；时间/距离准入及 TEASER/GICP 验收通过后才形成回环边。
            const auto candidates = std_manager_.SearchCandidates(descriptors); // 查看候选
            std::vector<int> tried;
            for (const auto &candidate : candidates)
            {
              const int h = candidate.match_id_.second;
              if (h < 0 || static_cast<std::size_t>(h) >= index)
                continue;
              double candidate_distance;
              double time_gap;
              {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                if (!loopCandidateEligible(msg->header.stamp, raw_pose, static_cast<std::size_t>(h))) // 时间检测
                  continue;
                candidate_distance = (frames_[h].odom_pose.translation() - raw_pose.translation()).norm();
                time_gap = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(frames_[h].stamp)).seconds();
              }
              if (std::any_of(tried.begin(), tried.end(), [h](int p)
                              { return std::abs(p - h) <= 2; }))
                continue;
              if (tried.size() >= static_cast<std::size_t>(registration_top_k_))
                break;
              tried.push_back(h);
              RegistrationCandidate match;
              match.historical_index = static_cast<std::size_t>(h);
              // STD retrieves places; a failed pose hint does not reject a candidate.
              // 三角形一致性给出 T_H_C 初值；缺少可靠初值仍可让 TEASER 无提示尝试配准。
              match.have_pose_hint = std_manager_.CandidatePoseHint(candidate, match.proposal);
              // Snapshot raw poses/clouds briefly; assembling/downsampling is a worker task.
              {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                const std::size_t first = h + 1 > submap_limits_.max_frames ? h + 1 - submap_limits_.max_frames : 0;
                match.target_window = SubmapWindow(submap_limits_);
                for (std::size_t j = first; j <= match.historical_index; ++j)
                {
                  const auto &f = frames_[j];
                  match.target_window.append({j, rclcpp::Time(f.stamp).seconds(),
                                              Eigen::Isometry3d(f.odom_pose.matrix()), f.cloud});
                }
              }
              if (pending_window.sharesFrames(match.target_window))
                continue;
              RCLCPP_INFO(get_logger(),
                          "STD %d -> %zu matched: time_gap=%.3f s (> %.3f) distance=%.2f m (<= %.2f) hint=%s std_pairs=%zu",
                          h, index, time_gap, loop_min_time_separation_s_, candidate_distance,
                          candidate_radius_m_, match.have_pose_hint ? "STD" : "none", candidate.match_list_.size());
              task.candidates.push_back(std::move(match)); // 添加配准任务
            }
          }
          else if (!std_manager_.plane_cloud_vec_.empty() &&
                   std_manager_.plane_cloud_vec_.back()->size() >= 3)
          {
            std::pair<int, double> result{-1, 0.0};
            RegistrationCandidate match;
            std::vector<std::pair<STDesc, STDesc>> pairs;
            std_manager_.SearchLoop(descriptors, result, match.proposal, pairs, nullptr,
                                    [this, &msg, &raw_pose](int historical)
                                    {
                                      std::lock_guard<std::mutex> lock(graph_mutex_);
                                      return historical >= 0 && loopCandidateEligible(msg->header.stamp, raw_pose,
                                                                                      static_cast<std::size_t>(historical));
                                    });
            if (result.first >= 0 && static_cast<std::size_t>(result.first) < index)
            {
              match.historical_index = static_cast<std::size_t>(result.first);
              // Direct GICP targets are local snapshots, assembled by the worker.
              const std::size_t half = static_cast<std::size_t>(std::max(0, gicp_submap_half_width_));
              const std::size_t first = match.historical_index > half ? match.historical_index - half : 0;
              const std::size_t last = std::min(index - 1, match.historical_index + half);
              std::lock_guard<std::mutex> lock(graph_mutex_);
              match.local_anchor = frames_[match.historical_index].odom_pose;
              match.odom_hint = match.local_anchor.between(raw_pose).matrix();
              match.local_frames.assign(frames_.begin() + first, frames_.begin() + last + 1);
              RCLCPP_INFO(get_logger(), "STD %zu -> %zu matched: score=%.3f", match.historical_index, index, result.second);
              task.candidates.push_back(std::move(match)); // 添加配准任务
            }
          }
        }
        adding_descriptors = true;
        std_manager_.AddSTDescs(descriptors); // 把描述子添加到集合中
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR(get_logger(), "STD failed on keyframe %u: %s", msg->id, error.what());
        if (adding_descriptors)
          backend_faulted_ = true;
        else
        {
          std_manager_.plane_cloud_vec_.resize(plane_count);
          std_manager_.corner_cloud_vec_.resize(corner_count);
        }
        return;
      }
    }
    const double std_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - std_begin)
                              .count();
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);

      // 帧：包含位姿、对应的点云
      Frame frame;
      frame.input_id = msg->id;
      frame.stamp = msg->header.stamp;
      frame.odom_pose = raw_pose;
      frame.cloud = cloud;
      frames_.push_back(std::move(frame));
      try
      {
        // Every ingested keyframe receives an odometry factor, regardless of queue overload.
        addGraphNode(index, raw_pose); // 添加因子节点
        publishCurrent(index, false);
        publishStdTriangles(index, descriptors);
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR(get_logger(), "Graph/publication failed on keyframe %u: %s", msg->id, error.what());
        backend_faulted_ = true;
        return;
      }
    }

    // 至此图节点已初始化，后台延迟结果只需连接既有节点，不需要重新插入节点初值。
    source_window_ = std::move(pending_window);
    last_input_id_ = msg->id;
    have_input_id_ = true;
    // 任务入队
    if (!task.candidates.empty())
      enqueueRegistration(std::move(task));
    std::size_t queued, dropped;
    {
      std::lock_guard<std::mutex> lock(registration_mutex_);
      queued = registration_tasks_.size();
      dropped = registration_tasks_dropped_;
    }
    const double total_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - processing_begin)
                                .count();
    double sensor_lag = 0.0;
    {
      std::lock_guard<std::mutex> lock(map_tf_mutex_);
      if (latest_data_stamp_)
        sensor_lag = (rclcpp::Time(*latest_data_stamp_) - rclcpp::Time(msg->header.stamp)).seconds();
    }
    RCLCPP_INFO_THROTTLE(get_logger(), diagnostics_clock_, 5000,
                         "Backend timing input=%u frames=%zu filter=%.1f ms descriptor=%.1f ms search_verify_insert=%.1f ms "
                         "ingest=%.1f ms sensor_lag=%.3f s registration_pending=%zu dropped_tasks=%zu",
                         msg->id, index + 1, filter_ms, descriptor_ms, std_ms - descriptor_ms, total_ms, sensor_lag, queued, dropped);
  }

  void LoopBackendNode::publishCurrent(std::size_t index, bool loop_accepted)
  {
    const Frame &frame = frames_[index];
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = frame.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "imu";
    odom.pose.pose = poseToMsg(frame.map_pose);
    odom_pub_->publish(odom);

    // Keyframe-only clients still work; live frontend stamps take precedence over delayed work.
    rememberDataStamp(frame.stamp);
    publishMapToWorld();

    publishPath(frame.stamp);
    if (loop_accepted || index % static_cast<std::size_t>(map_publish_stride_) == 0)
    {
      requestMap(frame.stamp);
    }
    if (loop_accepted)
      publishLoopMarkers(frame.stamp);
  }

  void LoopBackendNode::rememberDataStamp(const builtin_interfaces::msg::Time &stamp)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u)
      return;
    std::lock_guard<std::mutex> lock(map_tf_mutex_);
    if (!latest_data_stamp_ || rclcpp::Time(stamp) > rclcpp::Time(*latest_data_stamp_))
      latest_data_stamp_ = stamp;
  }

  void LoopBackendNode::onFrontendOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id == "world")
      rememberDataStamp(msg->header.stamp);
  }

  void LoopBackendNode::publishMapToWorld()
  {
    // TF 链为 map->world->imu：前端维持连续 world 轨迹，后端用 map->world 反映当前全局校正。
    // 历史各帧优化量通常不同；一条 TF 只能修正当前前端输出，历史地图仍须逐帧重建。
    std::lock_guard<std::mutex> lock(map_tf_mutex_);
    if (!have_map_transform_ || !latest_data_stamp_)
      return;
    if (published_map_tf_stamp_ && *published_map_tf_stamp_ == *latest_data_stamp_ && !map_tf_dirty_)
      return;
    const auto map_from_world = map_from_world_;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = *latest_data_stamp_;
    tf.header.frame_id = "map";
    tf.child_frame_id = "world";
    const auto transform = poseToMsg(map_from_world);
    tf.transform.translation.x = transform.position.x;
    tf.transform.translation.y = transform.position.y;
    tf.transform.translation.z = transform.position.z;
    tf.transform.rotation = transform.orientation;
    tf_broadcaster_->sendTransform(tf);
    published_map_tf_stamp_ = tf.header.stamp;
    map_tf_dirty_ = false;
  }

  void LoopBackendNode::publishPath(const builtin_interfaces::msg::Time &stamp)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "map";
    const std::size_t first = frames_.size() > static_cast<std::size_t>(path_max_poses_)
                                  ? frames_.size() - static_cast<std::size_t>(path_max_poses_)
                                  : 0;
    path.poses.reserve(frames_.size() - first);
    for (std::size_t i = first; i < frames_.size(); ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = frames_[i].stamp;
      pose.pose = poseToMsg(frames_[i].map_pose);
      path.poses.push_back(std::move(pose));
    }
    path_pub_->publish(path);
  }

  void LoopBackendNode::publishMap(const MapSnapshot &snapshot)
  {
    // 按每帧独立的优化位姿摆放原始机体系点云，才会得到随 PGO 形变后的历史地图。
    // 此处是显示地图，可按帧数/点数限流；完整地图导出使用独立的全量快照。
    const auto &frames = snapshot.frames;
    CloudPtr merged(new Cloud);
    const std::size_t limit = map_max_keyframes_ > 0
                                  ? static_cast<std::size_t>(map_max_keyframes_)
                                  : frames.size();
    const std::size_t first = frames.size() > limit ? frames.size() - limit : 0;
    for (std::size_t i = first; i < frames.size(); ++i)
    {
      const CloudPtr sampled = downsample(
          frames[i].cloud, 0.0,
          static_cast<std::size_t>(map_max_points_per_keyframe_));
      if (sampled->empty())
        continue;
      Cloud transformed;
      pcl::transformPointCloud(*sampled, transformed,
                               frames[i].map_pose.matrix().cast<float>());
      *merged += transformed;
    }
    const CloudPtr map = downsample(merged, map_voxel_size_, 0);
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*map, output);
    output.header.stamp = snapshot.stamp;
    output.header.frame_id = "map";
    map_pub_->publish(output);
  }

  void LoopBackendNode::publishStdTriangles(
      std::size_t index, const std::vector<STDesc> &descriptors)
  {
    using Marker = visualization_msgs::msg::Marker;
    Marker edges;
    edges.header.frame_id = "map";
    edges.header.stamp = frames_[index].stamp;
    edges.ns = "std_triangle_edges";
    edges.id = 0;
    edges.type = Marker::LINE_LIST;
    edges.action = Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = 0.08;
    edges.color.r = 0.1f;
    edges.color.g = 0.7f;
    edges.color.b = 1.0f;
    edges.color.a = 0.9f;

    Marker vertices = edges;
    vertices.ns = "std_triangle_vertices";
    vertices.type = Marker::SPHERE_LIST;
    vertices.scale.x = vertices.scale.y = vertices.scale.z = 0.18;
    vertices.color.r = 1.0f;
    vertices.color.g = 0.9f;
    vertices.color.b = 0.2f;

    if (std_visualization_max_triangles_ > 0)
    {
      const std::size_t limit =
          static_cast<std::size_t>(std_visualization_max_triangles_);
      const std::size_t step = std::max<std::size_t>(
          1, (descriptors.size() + limit - 1) / limit);
      const auto &pose = frames_[index].map_pose;
      for (std::size_t i = 0; i < descriptors.size(); i += step)
      {
        const auto &descriptor = descriptors[i];
        const Eigen::Vector3d *local[3] = {
            &descriptor.vertex_A_, &descriptor.vertex_B_, &descriptor.vertex_C_};
        if (!local[0]->allFinite() || !local[1]->allFinite() ||
            !local[2]->allFinite())
          continue;
        geometry_msgs::msg::Point corners[3];
        for (int j = 0; j < 3; ++j)
        {
          const Eigen::Vector3d point =
              pose.rotation().matrix() * *local[j] + pose.translation();
          corners[j].x = point.x();
          corners[j].y = point.y();
          corners[j].z = point.z();
          vertices.points.push_back(corners[j]);
        }
        for (int j = 0; j < 3; ++j)
        {
          edges.points.push_back(corners[j]);
          edges.points.push_back(corners[(j + 1) % 3]);
        }
      }
    }
    // Reuse fixed marker IDs so each keyframe replaces the previous triangles.
    // DELETE also clears an older display when this frame has no descriptors.
    if (edges.points.empty())
    {
      edges.action = Marker::DELETE;
      vertices.action = Marker::DELETE;
    }
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(edges));
    markers.markers.push_back(std::move(vertices));
    std_marker_pub_->publish(markers);
  }

  void LoopBackendNode::publishLoopMarkers(
      const builtin_interfaces::msg::Time &stamp)
  {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = stamp;
    line.ns = "accepted_loops";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.15;
    line.color.r = 0.0f;
    line.color.g = 0.9f;
    line.color.b = 0.3f;
    line.color.a = 1.0f;
    line.points.reserve(2 * loops_.size());
    for (const auto &[historical, current] : loops_)
    {
      for (const auto index : {historical, current})
      {
        const auto position = frames_[index].map_pose.translation();
        geometry_msgs::msg::Point point;
        point.x = position.x();
        point.y = position.y();
        point.z = position.z();
        line.points.push_back(point);
      }
    }
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(line));
    marker_pub_->publish(markers);
  }

  LoopBackendNode::~LoopBackendNode()
  {
    // Executor callbacks have stopped; finish queued/active registration before
    // taking the final snapshot. No ROS context is needed for disk export.
    if (save_map_on_shutdown_)
      waitForRegistration();
    stopping_ = true;
    registration_cv_.notify_all();
    map_cv_.notify_all();
    if (registration_thread_.joinable())
      registration_thread_.join();
    if (map_thread_.joinable())
      map_thread_.join();
    if (save_map_on_shutdown_)
    {
      std::string message;
      saveOptimizedMap(message);
    }
  }

  void LoopBackendNode::waitForRegistration()
  {
    std::unique_lock<std::mutex> lock(registration_mutex_);
    registration_idle_cv_.wait(lock, [this]
                               { return registration_tasks_.empty() && !registration_busy_; });
  }

  bool LoopBackendNode::saveOptimizedMap(std::string &message)
  {
    // 在图锁内冻结所有已保留关键帧的最终优化位姿，锁外逐点写文件，避免 I/O 长时间占图锁。
    try
    {
      std::vector<OptimizedMapFrame> snapshot;
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        if (backend_faulted_)
          throw std::runtime_error("Pose graph is faulted; refusing to export an inconsistent map");
        const auto estimate = isam_.calculateEstimate();
        snapshot.reserve(frames_.size());
        for (std::size_t i = 0; i < frames_.size(); ++i)
          snapshot.push_back({Eigen::Isometry3d(estimate.at<gtsam::Pose3>(i).matrix()), frames_[i].cloud});
      }
      const auto count = writeOptimizedMap(snapshot, save_map_path_, save_map_voxel_size_);
      message = "Saved " + std::to_string(snapshot.size()) + " keyframes, " +
                std::to_string(count) + " points in map frame to " + save_map_path_;
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
      return true;
    }
    catch (const std::exception &error)
    {
      message = error.what();
      RCLCPP_ERROR(get_logger(), "Optimized map save failed: %s", message.c_str());
      return false;
    }
  }

  void LoopBackendNode::enqueueRegistration(RegistrationTask task)
  {
    // 队列过载只丢最旧的待配准任务，frames_ 和里程计因子保留，避免轨迹图断链。
    task.enqueued = std::chrono::steady_clock::now(); // 入队时间
    {
      std::lock_guard<std::mutex> lock(registration_mutex_);
      if (stopping_)
        return;
      if (registration_tasks_.size() >= registration_queue_capacity_) // 大于了容量，就把最前面的任务弹出去
      {
        registration_tasks_.pop_front();
        ++registration_tasks_dropped_;
        RCLCPP_INFO_THROTTLE(get_logger(), diagnostics_clock_, 5000,
                             "Registration queue full: capacity=%zu dropped_tasks=%zu (keyframes retained)",
                             registration_queue_capacity_, registration_tasks_dropped_);
      }
      registration_tasks_.push_back(std::move(task)); // 任务入队
    }
    registration_cv_.notify_one(); // 唤醒配准线程
  }

  void LoopBackendNode::registrationWorker()
  {
    while (!stopping_)
    {
      RegistrationTask task;
      {
        // 等待配准任务的到来
        std::unique_lock<std::mutex> lock(registration_mutex_);
        registration_cv_.wait(lock, [this]
                              { return stopping_ || !registration_tasks_.empty(); });
        if (stopping_)
          return;
        task = std::move(registration_tasks_.front());
        registration_tasks_.pop_front();
        registration_busy_ = true;
      }
      const auto begin = std::chrono::steady_clock::now();
      const double queue_ms = std::chrono::duration<double, std::milli>(begin - task.enqueued).count();
      try
      {
        for (const auto &candidate : task.candidates)
        {
          if (stopping_ || backend_faulted_)
            break;
          {
            // 任务等待期间可能已有其他回环提交，重查准入及回环冷却间隔，避免密集重复加边。
            std::lock_guard<std::mutex> lock(graph_mutex_);
            if (!loopCandidateEligible(frames_[task.index].stamp, frames_[task.index].odom_pose,
                                       candidate.historical_index))
              continue;
            if (have_loop_ && (task.index <= last_loop_index_ ||
                               task.index - last_loop_index_ < static_cast<std::size_t>(loop_min_interval_)))
              break;
          }
          if (candidate_mode_ == "distance")
            RCLCPP_INFO(get_logger(), "Distance candidate %zu -> %zu: distance=%.2f m; verifying registration",
                        candidate.historical_index, task.index, candidate.proposal.first.norm());
          std::unique_ptr<RegistrationObservation> observation;

          // 两条几何验证路径最终都返回 T_H_C：多帧 TEASER+GICP，或 STD 初值直接 GICP。
          if (registration_mode_ == "std_teaser_gicp")
          {
            Eigen::Matrix4d hint = Eigen::Matrix4d::Identity();
            hint.block<3, 3>(0, 0) = candidate.proposal.second;
            hint.block<3, 1>(0, 3) = candidate.proposal.first;
            // 以多帧图组成子地图进行匹配，获取观测
            observation = verifySubmaps(task.source_window, candidate.target_window,
                                        task.current_cloud, candidate.historical_index,
                                        candidate.have_pose_hint ? &hint : nullptr);
          }
          else // 或者其他方法
          {
            CloudPtr merged(new Cloud);
            const auto cap = std::max<std::size_t>(100,
                                                   2 * registration_settings_.max_target / candidate.local_frames.size());
            for (const auto &frame : candidate.local_frames)
            {
              Cloud transformed;
              const auto local = downsample(frame.cloud, registration_settings_.voxel, cap);
              pcl::transformPointCloud(*local, transformed,
                                       candidate.local_anchor.between(frame.odom_pose).matrix().cast<float>().eval());
              *merged += transformed;
            }
            const auto target = downsample(merged, registration_settings_.voxel, registration_settings_.max_target);
            observation = verifyWithGicp(task.current_cloud, target, task.index,
                                         candidate.historical_index, candidate.proposal,
                                         candidate.odom_hint ? &*candidate.odom_hint : nullptr);
          }

          // 观测没有问题
          if (observation && !stopping_)
          {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            // 提交闭环，让 gtsam 进行 PGO 优化
            if (!backend_faulted_)
              commitLoop(task.index, candidate.historical_index, *observation);
            break;
          }
          // 预算只在候选之间检查，无法硬中断正在执行的单次 TEASER/GICP 求解。
          if (std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - begin)
                  .count() > registration_budget_ms_)
            break;
        }
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR(get_logger(), "Registration task %zu failed: %s", task.index, error.what());
      }
      // 普通日志汇报相关信息
      RCLCPP_INFO_THROTTLE(get_logger(), diagnostics_clock_, 5000,
                           "Registration timing frame=%zu queue_wait=%.1f ms work=%.1f ms",
                           task.index, queue_ms, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count());
      {
        std::lock_guard<std::mutex> lock(registration_mutex_);
        registration_busy_ = false;
      }
      registration_idle_cv_.notify_all();
    }
  }

  bool LoopBackendNode::loopTimeEligible(
      const builtin_interfaces::msg::Time &current, std::size_t historical) const
  {
    // 使用两帧采集时间差，而非后台入队/计算耗时；严格大于配置值，等于门限也拒绝。
    if (historical >= frames_.size())
      return false;
    // Subtract integer nanoseconds before comparing: exactly 30 s is rejected.
    const auto gap = (rclcpp::Time(current) - rclcpp::Time(frames_[historical].stamp)).nanoseconds();
    return static_cast<long double>(gap) >
           static_cast<long double>(loop_min_time_separation_s_) * 1000000000.0L;
  }

  bool LoopBackendNode::loopCandidateEligible(
      const builtin_interfaces::msg::Time &current, const gtsam::Pose3 &raw_pose,
      std::size_t historical) const
  {
    return loopTimeEligible(current, historical) &&
           (frames_[historical].odom_pose.translation() - raw_pose.translation()).norm() <= candidate_radius_m_;
  }

  void LoopBackendNode::commitLoop(std::size_t index, std::size_t historical,
                                   const RegistrationObservation &observation)
  {
    // 调用方持 graph_mutex_；配准的是旧任务帧 index，不应把最新位姿直接重置为这个旧帧。
    // 只加入 Z_H_C 约束，由整张图共同调整历史与当前节点。
    // graph_mutex_ held. A delayed result connects existing nodes; it never inserts
    // another initial value or resets the current pose to the old task's pose.
    if (index >= frames_.size() || historical >= index ||
        !loopCandidateEligible(frames_[index].stamp, frames_[index].odom_pose, historical))
      return;
    const auto begin = std::chrono::steady_clock::now();
    try
    {
      gtsam::NonlinearFactorGraph factors;
      // 因子残差对应 Log(Z_H_C^-1 * X_H^-1 * X_C)，测量方向不能取反。
      factors.add(gtsam::BetweenFactor<gtsam::Pose3>(historical, index,
                                                     observation.historical_from_current, loop_noise_));
      // Existing nodes were initialized from odometry before this delayed loop.
      // Refresh the robust weights/linearization, instead of relying on the
      // default ten-update cadence or on seeding a newly inserted loop node.
      gtsam::ISAM2UpdateParams parameters;
      // 回环会影响已有节点，强制重线性化/完整求解，并额外更新以传播修正。
      parameters.force_relinearize = true;
      parameters.forceFullSolve = true;
      isam_.update(factors, gtsam::Values{}, parameters);
      for (int iteration = 0; iteration < 3; ++iteration)
        isam_.update(gtsam::NonlinearFactorGraph{}, gtsam::Values{}, parameters);
      const auto estimate = isam_.calculateEstimate();
      // 必须刷新全部节点的 map_pose；只改回环两端会让轨迹和历史地图仍使用旧结果。
      for (std::size_t j = 0; j < frames_.size(); ++j)
        frames_[j].map_pose = estimate.at<gtsam::Pose3>(j);
    }
    catch (...)
    {
      backend_faulted_ = true;
      std::lock_guard<std::mutex> lock(map_tf_mutex_);
      have_map_transform_ = false;
      throw;
    }
    loops_.emplace_back(historical, index);
    last_loop_index_ = index;
    have_loop_ = true;
    const auto latest = frames_.size() - 1;
    {
      std::lock_guard<std::mutex> lock(map_tf_mutex_);
      // 使用最新图节点计算全局校正，而非使用刚完成配准的旧任务节点。
      map_from_world_ = frames_[latest].map_pose.compose(frames_[latest].odom_pose.inverse());
      have_map_transform_ = true;
      map_tf_dirty_ = true;
    }
    if (rclcpp::ok(get_node_base_interface()->get_context()))
      publishCurrent(latest, true);
    RCLCPP_INFO(get_logger(), "Loop %zu -> %zu %s: %s mse %.4f overlap %.3f graph=%.1f ms latest=%zu",
                historical, index, decisionText(true, "accepted").c_str(),
                registration_mode_ == "std_teaser_gicp" ? "TEASER/GICP" : "GICP",
                observation.fitness, observation.overlap, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count(), latest);
  }

  void LoopBackendNode::requestMap(const builtin_interfaces::msg::Time &stamp)
  {
    // 图锁内只复制位姿和共享点云指针；锁外地图线程负责耗时拼接，待处理快照只保留最新一份。
    // Called under graph_mutex_. Shared cloud pointers are immutable after ingest.
    MapSnapshot snapshot;
    const std::size_t limit = map_max_keyframes_ > 0 ? static_cast<std::size_t>(map_max_keyframes_) : frames_.size();
    const std::size_t first = frames_.size() > limit ? frames_.size() - limit : 0;
    snapshot.frames.assign(frames_.begin() + first, frames_.end());
    snapshot.stamp = stamp;
    snapshot.enqueued = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      pending_map_ = std::move(snapshot); // Coalesce into the newest pending snapshot.
    }
    map_cv_.notify_one();
  }

  void LoopBackendNode::mapWorker()
  {
    while (!stopping_)
    {
      MapSnapshot snapshot;
      {
        std::unique_lock<std::mutex> lock(map_mutex_);
        map_cv_.wait(lock, [this]
                     { return stopping_ || pending_map_.has_value(); });
        if (stopping_)
          return;
        snapshot = std::move(*pending_map_);
        pending_map_.reset();
      }
      const auto begin = std::chrono::steady_clock::now();
      try
      {
        publishMap(snapshot);
        RCLCPP_INFO_THROTTLE(get_logger(), diagnostics_clock_, 5000,
                             "Map timing frames=%zu queue_wait=%.1f ms rebuild=%.1f ms", snapshot.frames.size(),
                             std::chrono::duration<double, std::milli>(begin - snapshot.enqueued).count(),
                             std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count());
      }
      catch (const std::exception &error)
      {
        if (!stopping_)
          RCLCPP_ERROR(get_logger(), "Map rebuild failed: %s", error.what());
      }
    }
  }

} // namespace super_lio_slam
