#include "lio/super_lio_reloc.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

using namespace BASIC;
namespace LI2Sup
{
  using namespace super_lio_slam;

  namespace
  {
    // Highlight decision fields only; leave numeric diagnostics in the terminal's normal color.
    std::string decisionText(bool accepted, const std::string &text)
    {
      return std::string(accepted ? "\033[32m" : "\033[31m") + text + "\033[0m";
    }
  } // namespace

  SuperLIOReLoc::~SuperLIOReLoc()
  {
    stopping_ = true;
    registration_cv_.notify_all();
    if (registration_thread_.joinable())
      registration_thread_.join();
  }

  void SuperLIOReLoc::setStatus(const std::string &status)
  {
    if (status_ == status)
      return;
    status_ = status;
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(data_wrapper_->get_logger(), "Relocation state: %s", status.c_str());
  }

  void SuperLIOReLoc::init()
  {
    // 阅读主线：init 加载参考地图 -> process 局部 LIO -> enqueueRegistration 冻结快照
    // -> registrationWorker 求 T_W_S -> commitRegistration 验证并接管最新 ESKF 状态。
#ifndef SUPERLIO_WITH_TEASER
    throw std::runtime_error("Motion-compensated relocation requires SUPERLIO_WITH_TEASER=ON");
#endif
    SuperLIO::init();
    keyframe_enabled_ = false; // Never mix local-S keyframes into the world-frame loop graph.
    auto &node = *data_wrapper_;
    debug_snapshot_dir_ = node.declare_parameter<std::string>("relocation.debug_snapshot_dir", "");
    auto positive = [&](const char *key, double value)
    {
      const auto result = node.declare_parameter<double>(key, value);
      if (!std::isfinite(result) || result <= 0)
        throw std::invalid_argument(std::string(key) + " must be finite and positive");
      return result;
    };
    auto integer = [&](const char *key, int value, int low, int high)
    {
      const auto result = node.declare_parameter<int>(key, value);
      if (result < low || result > high)
        throw std::invalid_argument(std::string(key) + " is outside its supported range");
      return result;
    };
    registration_min_frames_ = integer("relocation.registration_min_frames", 5, 1, 100000);
    for (const auto &[name, value] : node.get_node_parameters_interface()->get_parameter_overrides())
      if (name.rfind("relocation.window_", 0) == 0 || name.rfind("relocation.keyframe_", 0) == 0)
        RCLCPP_WARN(node.get_logger(), "%s is obsolete and ignored: relocation registers the cumulative S-frame LIO map", name.c_str());
    imu_samples_ = integer("relocation.imu_init_samples", 50, 20, 10000);
    init_duration_ = positive("relocation.imu_init_duration_s", .3);
    max_gyro_ = positive("relocation.imu_max_gyro_rad_s", .15);
    max_gyro_std_ = positive("relocation.imu_max_gyro_std", .02);
    max_accel_std_ = positive("relocation.imu_max_accel_std", .15);
    min_plane_points_ = integer("relocation.local_min_plane_points", 30, 6, 20000);
    registration_interval_ = positive("relocation.registration_interval_s", 1.0);
    crop_radius_ = node.declare_parameter<double>("relocation.map_crop_radius_m", 80.0);
    if (!std::isfinite(crop_radius_) || crop_radius_ < 0)
      throw std::invalid_argument("map crop radius must be nonnegative");
    gravity_angle_ = positive("relocation.max_gravity_angle_deg", 10.0);
    if (gravity_angle_ > 180)
      throw std::invalid_argument("gravity angle must be <=180 degrees");
    const auto position_sigma = positive("relocation.registration_position_sigma_m", .2);
    const auto rotation_sigma = positive("relocation.registration_rotation_sigma_deg", 2.0) * M_PI / 180.0;
    position_variance_ = position_sigma * position_sigma;
    rotation_variance_ = rotation_sigma * rotation_sigma;
    have_prior_ = node.declare_parameter<bool>("relocation.use_initial_pose", true);
    const M3 R = (Eigen::AngleAxisf(g_init_yaw * M_PI / 180.0, V3::UnitZ()) *
                  Eigen::AngleAxisf(g_init_pitch * M_PI / 180.0, V3::UnitY()) *
                  Eigen::AngleAxisf(g_init_roll * M_PI / 180.0, V3::UnitX()))
                     .toRotationMatrix();
    prior_pose_ = SE3(R, V3(g_init_px, g_init_py, g_init_pz));

    auto &s = settings_;
    s.voxel = positive("registration_voxel_size_m", .3);
    s.min_points = integer("registration_min_points", 100, 30, 100000);
    // 0 keeps every voxel-filtered point. Positive limits sample the complete
    // spatial map, never truncate its history to a recent scan window.
    s.max_source = integer("registration_max_source_points", 0, 0, 10000000);
    s.max_target = integer("registration_max_target_points", 50000, 0, 10000000);
    s.normal_radius = positive("fpfh_normal_radius_m", .9);
    s.feature_radius = positive("fpfh_feature_radius_m", 1.5);
    s.feature_matching = node.declare_parameter<std::string>("fpfh_matching_mode", "nearest");
    if (s.feature_matching != "nearest" && s.feature_matching != "mutual_ratio")
      throw std::invalid_argument("fpfh_matching_mode must be nearest or mutual_ratio");
    s.feature_ratio = positive("fpfh_distance_ratio_threshold", .9);
    s.hint_radius = positive("fpfh_pose_hint_radius_m", 3.0);
    s.min_pairs = integer("correspondence_min_unique", 6, 3, 500);
    s.max_pairs = integer("correspondence_max_unique", 2000, 3, 10000);
    s.min_baseline = positive("correspondence_min_baseline_m", 2.0);
    s.noise_bound = positive("teaser_noise_bound_m", .3);
    s.rotation_iterations = integer("teaser_rotation_max_iterations", 100, 1, 1000);
    s.clique_seconds = positive("teaser_max_clique_time_limit_s", 30.0);
    const auto clique_mode = node.declare_parameter<std::string>("teaser_inlier_selection", "pmc_exact");
    if (clique_mode != "pmc_exact" && clique_mode != "pmc_heuristic")
      throw std::invalid_argument("teaser_inlier_selection must be pmc_exact or pmc_heuristic");
    s.exact_clique = clique_mode == "pmc_exact";
    s.rotation_gnc_factor = positive("teaser_rotation_gnc_factor", 1.4);
    s.rotation_cost_threshold = positive("teaser_rotation_cost_threshold", 1e-4);
    if (s.rotation_gnc_factor <= 1)
      throw std::invalid_argument("teaser_rotation_gnc_factor must be greater than one");
    s.threads = integer("teaser_max_clique_num_threads", 4, 1, 16);
    s.coarse_radius = positive("coarse_residual_threshold_m", .6);
    s.coarse_min_inliers = integer("coarse_min_inliers", 6, 3, 500);
    s.coarse_min_ratio = positive("coarse_min_inlier_ratio", .01);
    s.coarse_max_rmse = positive("coarse_max_inlier_rmse_m", .4);
    s.max_iterations = integer("gicp_max_iterations", 40, 1, 200);
    s.covariance_neighbors = integer("gicp_correspondence_randomness", 20, 4, 100);
    s.correspondence_distance = positive("gicp_max_correspondence_distance_m", 2.0);
    s.quality_radius = positive("quality_nn_radius_m", 1.0);
    s.min_overlap = positive("quality_min_source_overlap", .8);
    s.min_current_overlap = positive("quality_min_current_frame_overlap", .8);
    s.min_inliers = integer("quality_min_inlier_count", 100, 3, 100000);
    s.max_mse = positive("quality_max_inlier_mse_m2", .1);
    s.min_information_ratio = positive("quality_min_information_ratio", 1e-5);
    s.max_translation = positive("quality_max_refinement_translation_m", 5.0);
    s.max_rotation_deg = positive("quality_max_refinement_rotation_deg", 20.0);
    if (s.feature_radius <= s.normal_radius || s.feature_ratio > 1 || s.min_pairs > s.max_pairs ||
        s.coarse_min_ratio > 1 || s.min_overlap >= 1 || s.min_current_overlap >= 1)
      throw std::invalid_argument("inconsistent relocation feature/quality parameters");
    rclcpp::PublisherOptions status_options;
    status_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    status_pub_ = node.create_publisher<std_msgs::msg::String>("/lio/relocation/status",
                                                               rclcpp::QoS(1).reliable().transient_local(), status_options);
    visualize_local_map_ = node.declare_parameter<bool>("relocation.visualize_local_map", true);
    local_map_preview_interval_ = positive("relocation.local_map_visualization_interval_s", 0.5);
    local_map_preview_max_points_ = integer("relocation.local_map_visualization_max_points", 50000, 100, 1000000);
    local_map_preview_pub_ = node.create_publisher<visualization_msgs::msg::Marker>(
        "/lio/relocation/local_map_preview", rclcpp::QoS(1).reliable().transient_local(), status_options);
    setStatus("LOAD_REFERENCE_MAP");
    std::string map_path = node.declare_parameter<std::string>("relocation.map_path", "");
    if (map_path.empty())
      map_path = g_save_map_dir + "/" + g_map_name;
    if (!debug_snapshot_dir_.empty() &&
        std::filesystem::weakly_canonical(std::filesystem::path(debug_snapshot_dir_) / "source.pcd") ==
            std::filesystem::weakly_canonical(map_path))
      throw std::invalid_argument("Debug snapshot source.pcd must not overwrite the reference map");
    reference_cloud_.reset(new PointCloudType);
    if (pcl::io::loadPCDFile<PointType>(map_path, *reference_cloud_) < 0)
      throw std::runtime_error("Cannot load relocation reference map: " + map_path);
    const auto loaded_points = reference_cloud_->size();
    const auto reference_voxel = node.declare_parameter<double>("relocation.reference_map_voxel_size_m", s.voxel);
    // 导入时统一过滤，显示、参考地图查询和后台配准共用该点云，不修改磁盘原始 PCD。
    reference_cloud_ = super_lio_slam::prepareRelocationReference(reference_cloud_, reference_voxel);
    VV3 points;
    points.reserve(reference_cloud_->size());
    for (const auto &p : *reference_cloud_)
      points.push_back(p.getVector3fMap());
    if (points.size() < static_cast<std::size_t>(s.min_points))
      throw std::runtime_error("Relocation reference map is empty/sparse");
    reference_ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
    reference_ivox_->insert(points);
    reference_tree_.setInputCloud(reference_cloud_);
    // 局部地图与先验地图并存：未定位时 Observe() 只查询局部地图，成功后再切换 ivox_。
    local_ivox_ = ivox_;
    data_wrapper_->set_global_map(reference_cloud_);

    // 设置初始化位姿猜测回调函数
    data_wrapper_->setRelocationInitialPoseCallback([this](const auto &msg)
                                                    { receivePrior(msg); });
    setStatus("WAITING_FOR_STATIONARY_IMU");
    registration_thread_ = std::thread(&SuperLIOReLoc::registrationWorker, this);
    RCLCPP_INFO(node.get_logger(), "Relocation reference loaded: %zu -> %zu points, voxel=%.3f m, %s",
                loaded_points, points.size(), reference_voxel, map_path.c_str());
    RCLCPP_INFO(node.get_logger(), "Relocation coarse profile: matcher=%s pairs<=%d clique=%s budget=%.2f s voxel=%.2f normal=%.2f feature=%.2f",
                s.feature_matching.c_str(), s.max_pairs, clique_mode.c_str(), s.clique_seconds,
                s.voxel, s.normal_radius, s.feature_radius);
  }

  bool SuperLIOReLoc::initializeLocal()
  {
    // 样本数、持续时间和 IMU 波动共同把关；仅样本充足不能证明适合做静止初始化。
    for (const auto &imu : measures_.imu)
    {
      if (!imu.acc.allFinite() || !imu.gyr.allFinite() || imu.gyr.norm() > max_gyro_)
      {
        initialization_imu_.clear();
        continue;
      }
      initialization_imu_.push_back(imu);
    }
    // 剔除过多的 imu 采样数据
    while (initialization_imu_.size() > std::max<std::size_t>(imu_samples_ * 4, 1000))
      initialization_imu_.pop_front();
    RCLCPP_INFO_THROTTLE(data_wrapper_->get_logger(), *data_wrapper_->get_clock(), 5000,
                         "Relocation IMU initialization: samples=%zu scan_points=%zu", initialization_imu_.size(), measures_.lidar.pc->size());
    // 防止过少的 imu 采样数据
    if (initialization_imu_.size() < imu_samples_ ||
        initialization_imu_.back().secs - initialization_imu_.front().secs < init_duration_)
      return false;
    // 均值
    V3 mean_acc = V3::Zero(), mean_gyro = V3::Zero(), acc_square = V3::Zero(), gyro_square = V3::Zero();
    for (const auto &imu : initialization_imu_)
    {
      mean_acc += imu.acc;
      mean_gyro += imu.gyr;
      acc_square += imu.acc.cwiseProduct(imu.acc);
      gyro_square += imu.gyr.cwiseProduct(imu.gyr);
    }
    const float n = initialization_imu_.size();
    // 均值
    mean_acc /= n;
    mean_gyro /= n;
    // 三轴方差之和开方是整体波动尺度，用来排除明显运动；不参与后续配准评分。
    const float acc_std = (acc_square / n - mean_acc.cwiseProduct(mean_acc)).cwiseMax(0).sum();
    const float gyro_std = (gyro_square / n - mean_gyro.cwiseProduct(mean_gyro)).cwiseMax(0).sum();
    if (std::sqrt(acc_std) > max_accel_std_ || std::sqrt(gyro_std) > max_gyro_std_ || mean_acc.norm() < 1e-3)
    {
      initialization_imu_.clear();
      return false;
    }
    // 将静止比力方向对齐到 S 系 +Z，得到重力对齐姿态；此时仍不知道 S 相对 W 的位置和偏航。
    const M3 rotation = Quat::FromTwoVectors(mean_acc, V3::UnitZ()).toRotationMatrix();
    ESKF::Options options;
    options.gyro_var_ = g_imu_ng;
    options.acce_var_ = g_imu_na;
    options.bias_gyro_var_ = g_imu_nbg;
    options.bias_acce_var_ = g_imu_nba;
    options.num_iterations_ = g_kf_max_iterations;
    options.quit_eps_ = g_kf_quit_eps;

    // kf 初始化
    kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), g_gravity_norm / mean_acc.norm(), V3(0, 0, -g_gravity_norm));

    // 设置初始预测
    kf_->InitializePrediction(SysState(measures_.lidar.end_time, SO3(rotation)), initialization_imu_.back());
    scan_undistort_full_->clear();

    // Only one stationary scan seeds the local map. Moving scans use propagation/deskew below.
    for (const auto &p : *measures_.lidar.pc)
    {
      const V3 body = g_lidar_imu * V3(p.x, p.y, p.z);
      PointType out;
      out.getVector3fMap() = body;
      out.intensity = p.intensity;
      scan_undistort_full_->push_back(out);
    }

    // 降采样
    DownSample();

    // 扫描限制
    limitScan();

    // 设置剩下的数据和状态
    if (ds_undistort_->size() < static_cast<std::size_t>(min_plane_points_))
    {
      kf_->init_ = false;
      return false;
    }
    VV3 points;
    for (const auto &p : *ds_undistort_)
      points.push_back(rotation * p.getVector3fMap());
    local_ivox_->insert(points);
    // 系统初始位姿
    sys_init_pose_ = kf_->GetSE3();
    prior_local_pose_ = sys_init_pose_;
    // 状态变化
    stage_ = Stage::Local;
    tracking_good_ = true;
    rememberLocalPose();
    initialization_imu_.clear();
    setStatus("LOCAL_TRACKING");
    Output();
    return true;
  }

  void SuperLIOReLoc::limitScan()
  {
    if (ds_undistort_->size() <= 20000)
      return;
    CloudPtr capped(new PointCloudType);
    for (std::size_t i = 0; i < 20000; ++i)
      capped->push_back((*ds_undistort_)[i * ds_undistort_->size() / 20000]);
    ds_undistort_ = capped;
  }

  void SuperLIOReLoc::rememberLocalPose()
  {
    // 该 deque 只为带时间戳的 /initialpose 查找同一时刻局部位姿，不存点云，也不裁剪配准地图。
    ++local_frame_count_;
    prior_pose_history_.push_back({measures_.lidar.end_time, kf_->GetSE3()});
    // This small pose-only history supports stamped /initialpose messages.
    // It contains no clouds and has no effect on registration map coverage.
    while (prior_pose_history_.size() > 1 &&
           (prior_pose_history_.back().stamp - prior_pose_history_.front().stamp > 5.0 ||
            prior_pose_history_.size() > 2000))
      prior_pose_history_.pop_front();
  }

  void SuperLIOReLoc::enqueueRegistration()
  {
    // 按成功跟踪帧数和传感器时间节流；地图包含体素容器中全部保留点，不是最近 N 帧窗口。
    if (local_frame_count_ < static_cast<std::size_t>(registration_min_frames_) ||
        (last_task_time_ >= 0 && measures_.lidar.end_time - last_task_time_ < registration_interval_))
      return;
    RelocationSnapshot task;
    task.generation = generation_;
    task.stamp = measures_.lidar.end_time;
    task.frame_count = local_frame_count_;
    // 冻结同一时刻的地图、扫描和 T_S_IM，防止后台把新位姿误配给旧扫描。
    task.local_from_current = Eigen::Isometry3d(kf_->GetSE3().matrix().cast<double>());
    task.current_cloud.reset(new PointCloudType(*ds_undistort_));
    // The sensor callback owns local_ivox_. Freeze the whole retained S-frame
    // map here; the registration worker never reads a map being updated by LIO.
    std::vector<float> xyz;
    local_ivox_->getMap(xyz);
    CloudPtr local_map(new PointCloudType);
    local_map->reserve(xyz.size() / 3);
    for (std::size_t i = 0; i + 2 < xyz.size(); i += 3)
    {
      PointType point;
      point.x = xyz[i]; point.y = xyz[i + 1]; point.z = xyz[i + 2];
      point.intensity = 0; // OctVoxMap stores geometry only; FPFH/GICP use XYZ.
      local_map->push_back(point);
    }
    task.local_map = local_map;
    task.reference = reference_cloud_;
    task.local_gravity = kf_->GetGravity().cast<double>();
    // 先验给的是某帧 T_W_I，与对应 T_S_I 相消后得到整张局部地图的 T_W_S 提示。
    if (have_prior_)
      task.world_from_local_hint = (prior_pose_ * prior_local_pose_.inverse()).matrix().cast<double>();
    {
      std::lock_guard<std::mutex> lock(registration_mutex_);
      // 替换尚未运行的旧快照；正在求解的快照保持不变，前端继续累积地图和估计位姿。
      pending_task_ = std::move(task); // Latest pending map wins; running solver is not interrupted.
    }
    last_task_time_ = measures_.lidar.end_time;
    registration_cv_.notify_one();
  }

  // 配准重定位线程
  void SuperLIOReLoc::registrationWorker()
  {
    // 后台只读不可变快照并产出结果；不直接访问/修改 ESKF 或前端局部地图。
    // 求解时不持 registration_mutex_，耗时 TEASER/GICP 不应阻塞前端投递和取回结果。
    while (!stopping_)
    {
      RelocationSnapshot task;
      {
        // 接收任务并且执行配准任务
        std::unique_lock<std::mutex> lock(registration_mutex_);
        registration_cv_.wait(lock, [this]
                              { return stopping_ || pending_task_.has_value(); });
        if (stopping_)
          return;
        task = std::move(*pending_task_);
        pending_task_.reset();
      }
      RelocationResult result;
      if (!debug_snapshot_dir_.empty())
      {
        // Optional, bounded on-disk reproduction of the ACTUAL online input.
        // One worker overwrites the latest pair; no per-frame bag recording.
        try
        {
          const auto directory = std::filesystem::path(debug_snapshot_dir_);
          std::filesystem::create_directories(directory);
          const auto temporary = directory / ".source.tmp.pcd";
          if (pcl::io::savePCDFileBinary(temporary.string(), *task.local_map) < 0)
            throw std::runtime_error("Cannot write source.pcd");
          std::filesystem::rename(temporary, directory / "source.pcd");
          std::ofstream metadata(directory / ".snapshot.tmp.json");
          metadata.exceptions(std::ios::failbit | std::ios::badbit);
          metadata << std::setprecision(17) << "{\"stamp\":" << task.stamp
                   << ",\"frames\":" << task.frame_count << ",\"map_points\":" << task.local_map->size()
                   << ",\"local_from_current\":[";
          for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col)
              metadata << (row || col ? "," : "") << task.local_from_current.matrix()(row, col);
          metadata << "]}\n";
          metadata.close();
          std::filesystem::rename(directory / ".snapshot.tmp.json", directory / "snapshot.json");
        }
        catch (const std::exception &error)
        {
          RCLCPP_WARN(data_wrapper_->get_logger(), "Cannot export relocation debug snapshot: %s", error.what());
        }
      }
      try
      {
        // 重定位配准
        result = registerRelocation(task, settings_, crop_radius_, gravity_angle_);
      }
      catch (const std::exception &e)
      {
        result.generation = task.generation;
        result.stamp = task.stamp;
        result.registration.reason = std::string("exception: ") + e.what();
      }
      const auto &r = result.registration;
      RCLCPP_INFO(data_wrapper_->get_logger(),
                  "Relocation registration %s: generation=%lu stamp=%.6f frames=%zu map_points=%zu scan_points=%zu coarse=%zu/%zu rmse=%.4f "
                  "mse=%.4f overlap=%.3f current_overlap=%.3f stage=%s",
                  decisionText(r.accepted, r.accepted ? "accepted" : "rejected").c_str(),
                  static_cast<unsigned long>(result.generation), result.stamp, task.frame_count,
                  task.local_map->size(), task.current_cloud->size(), r.coarse_inliers, r.pairs,
                  r.coarse_rmse, r.mse, r.overlap, r.current_overlap, decisionText(r.accepted, r.reason).c_str());
      std::lock_guard<std::mutex> lock(registration_mutex_);
      // 成功结果需留给前端进行最新扫描复查，不能被稍后返回的失败结果覆盖。
      // A newer rejected task must not overwrite a successful older snapshot
      // before the frontend gets a chance to align it to the latest scan.
      if (!stopping_ && (!pending_result_ || !pending_result_->registration.accepted ||
                         result.registration.accepted || pending_result_->generation != result.generation))
        pending_result_ = std::move(result);
    }
  }

  // 在传感器处理流程中提交配准结果；后台线程不负责切换滤波器状态。
  bool SuperLIOReLoc::commitRegistration()
  {
    std::optional<RelocationResult> result;
    {
      // 重置结果
      std::lock_guard<std::mutex> lock(registration_mutex_);
      result = std::move(pending_result_);
      pending_result_.reset();
    }
    // 上一个结果是否有效，无效就返回让循环提交下一次配准
    if (!result || result->generation != generation_ || !result->registration.accepted || !tracking_good_)
      return false;
    const auto &transform = result->registration.transform;
    if (!validRigidTransform(transform))
      return false;
    // 快照 M 求得 T_W_S 后，直接乘当前 T_S_IN；无需把滤波器退回 M，也不能重置时间戳。
    // 再用最新 N 帧扫描验证，避免“旧地图能对齐，但车辆当前已跟丢”仍被接管。
    // A delayed snapshot must also align the latest moving scan before the handoff.
    const Eigen::Matrix4f current = (transform * kf_->GetSE3().matrix().cast<double>()).cast<float>();
    CloudPtr aligned(new PointCloudType);
    pcl::transformPointCloud(*ds_undistort_, *aligned, current);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    std::size_t inliers = 0;
    double sum = 0;
    for (const auto &p : *aligned)
    {
      if (reference_tree_.nearestKSearch(p, 1, indices, distances) == 1 && distances[0] <= settings_.quality_radius * settings_.quality_radius)
      {
        ++inliers;
        sum += distances[0];
      }
    }
    const double overlap = aligned->empty() ? 0 : static_cast<double>(inliers) / aligned->size();
    const double mse = inliers ? sum / inliers : std::numeric_limits<double>::infinity();
    if (inliers < static_cast<std::size_t>(settings_.min_inliers) || overlap <= settings_.min_current_overlap || mse >= settings_.max_mse)
    {
      const char *reason = inliers < static_cast<std::size_t>(settings_.min_inliers) ? "few_current_inliers"
                           : overlap <= settings_.min_current_overlap                ? "low_current_overlap"
                                                                                     : "high_inlier_mse";
      RCLCPP_WARN(data_wrapper_->get_logger(),
                  "Delayed relocation %s at latest scan: inliers=%zu (min %d) mse=%.4f (max %.4f) "
                  "overlap=%.3f (min %.3f) stage=%s",
                  decisionText(false, "rejected").c_str(), inliers, settings_.min_inliers, mse, settings_.max_mse,
                  overlap, settings_.min_current_overlap, decisionText(false, reason).c_str());
      return false;
    }
    const SE3 world_from_local(transform.cast<scalar>());
    const double latest_time = kf_->GetTime();

    // 一次性完成参考系接管：位姿、速度、重力、协方差以及高频预测缓存一起变换。
    // 偏置保持机体系定义，时间保留最新扫描时刻；具体雅可比见 ESKF::ChangeReferenceFrame。
    kf_->ChangeReferenceFrame(world_from_local, position_variance_, rotation_variance_);
    sys_init_pose_ = world_from_local * sys_init_pose_;
    last_pose_ = kf_->GetSE3();
    propagate_states_.clear();
    points_world_v3_.clear();
    world_pc_->clear();
    ds_world_->clear();
    // 从下一次 Observe 起改查参考地图；释放前端局部地图、清除旧坐标点缓存。
    // 已经运行的后台任务仍持有自己的不可变点云快照，结束后自然释放。
    ivox_ = reference_ivox_;
    local_ivox_.reset();

    // 设置相关信号
    // ROS 前后均使用 world 标签：此处清空局部 Path 和黄色预览，随后 Output 发布真实 W 系结果。
    data_wrapper_->commitRelocationFrame(kf_->GetNavState().timestamp);
    publishLocalMapPreview(true);
    stage_ = Stage::World;
    ++generation_;
    stopping_ = true;
    registration_cv_.notify_all();
    prior_pose_history_.clear();
    {
      std::lock_guard<std::mutex> lock(registration_mutex_);
      pending_task_.reset();
      pending_result_.reset();
    }
    setStatus("WORLD_MAP_TRACKING");
    RCLCPP_INFO(data_wrapper_->get_logger(),
                "Relocation committed: snapshot=%.6f latest=%.6f preserved_time=%.6f mse=%.4f overlap=%.3f %s stage=%s",
                result->stamp, measures_.lidar.end_time, latest_time, mse, overlap,
                decisionText(true, "accepted").c_str(), decisionText(true, "latest_scan_quality_passed").c_str());
    return true;
  }

  // 接收先验参数
  void SuperLIOReLoc::receivePrior(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    if (stage_ == Stage::World || stage_ == Stage::Failed)
    {
      RCLCPP_WARN(data_wrapper_->get_logger(), "Relocation already completed/failed; restart for a new localization session");
      return;
    }
    if (!msg.header.frame_id.empty() && msg.header.frame_id != "world")
    {
      RCLCPP_WARN(data_wrapper_->get_logger(), "Initial pose must be expressed in world");
      return;
    }
    // 位姿分解
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;

    // 旋转
    Quat rotation(q.w, q.x, q.y, q.z);
    if (!V3(p.x, p.y, p.z).allFinite() || !rotation.coeffs().allFinite() || rotation.norm() < 1e-6)
      return;
    // 局部位姿
    SE3 local_pose = kf_->GetSE3();
    if (stage_ == Stage::Local && (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0))
    {
      const double stamp = data_wrapper_->sensorTime(msg.header.stamp);
      auto best = prior_pose_history_.end();
      double gap = .25;
      for (auto it = prior_pose_history_.begin(); it != prior_pose_history_.end(); ++it)
        if (std::abs(it->stamp - stamp) <= gap)
        {
          best = it;
          gap = std::abs(it->stamp - stamp);
        }
      if (best == prior_pose_history_.end())
      {
        RCLCPP_WARN(data_wrapper_->get_logger(), "Initial pose timestamp has no matching local pose (tolerance 0.25 s)");
        return;
      }
      local_pose = best->pose;
    }
    // 四元数归一化
    rotation.normalize();
    // 先验位姿
    prior_pose_ = SE3(rotation, V3(p.x, p.y, p.z));
    prior_local_pose_ = local_pose;
    have_prior_ = true;
    // 新先验使旧任务过期；后台无需强制中断，但返回结果的 generation 必须与当前一致。
    ++generation_;
    last_task_time_ = -1;
    std::lock_guard<std::mutex> lock(registration_mutex_);
    pending_task_.reset();
    pending_result_.reset();
  }

  void SuperLIOReLoc::process()
  {
    // imu 和 lidar 时钟同步
    if (!data_wrapper_->sync_measure(measures_) || stage_ == Stage::Failed)
      return;
    if (!std::isfinite(measures_.lidar.end_time) || measures_.lidar.end_time <= last_scan_time_)
    {
      stage_ = Stage::Failed;
      setStatus("ERROR_SENSOR_TIME_ROLLBACK");
      return;
    }
    last_scan_time_ = measures_.lidar.end_time;

    // 如果当前系统刚启动，那就是初始化局部地图和位姿
    if (stage_ == Stage::Bootstrap)
    {
      initializeLocal();
      return;
    }

    // Local 阶段查询局部地图，World 阶段查询参考地图；共用同一套 IMU 传播与点到面更新。
    Propagation_Undistort(); // 去畸变
    DownSample();            // 降采样
    limitScan();             // 扫描限制
    Observe();               // 观测更新
    tracking_good_ = 0;
    int planes = 0;
    for (std::size_t i = 0; i < ds_undistort_->size(); ++i)
      if (effect_mask_[i])
        ++planes;
    tracking_good_ = planes >= min_plane_points_ && kf_->GetSE3().matrix().allFinite();
   
    if (stage_ == Stage::Local) // 如果是局部建图，就尝试进行重定位
    {
      if (tracking_good_)
      {
        SuperLIO::UpdateMap(); // 地图更新
        rememberLocalPose();
        // 没有可提交结果也会返回 false；不等同于“后台刚刚配准失败”。先尝试接管，再投递快照。
        if (!commitRegistration())
        {
          enqueueRegistration(); // 启动新的配准
          setStatus("LOCAL_TRACKING");
        }
      }
      else
      {
        setStatus("LOCAL_TRACKING_DEGRADED");
      }
    }
    else if (g_update_map && tracking_good_)  // 如果是全局坐标系建图，那就更新地图
    {
      SuperLIO::UpdateMap(); // 如果可以更新且跟踪良好，那就更新
    }
    Output();
  }

  void SuperLIOReLoc::publishLocalMapPreview(bool clear)
  {
    if (!visualize_local_map_)
      return;
    using Marker = visualization_msgs::msg::Marker;
    Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "relocation_local_map_preview";
    marker.id = 0;
    marker.action = clear ? Marker::DELETE : Marker::ADD;
    marker.type = Marker::POINTS;
    marker.pose.orientation.w = 1.0;
    if (!clear)
    {
      if (stage_ != Stage::Local || !local_ivox_ ||
          local_map_preview_pub_->get_subscription_count() == 0 ||
          (last_local_map_preview_time_ >= 0 &&
           measures_.lidar.end_time - last_local_map_preview_time_ < local_map_preview_interval_))
        return;
      // Match the provisional odometry/scan coordinates shown directly in
      // world. A registration prior must not move only the preview map.
      std::vector<float> xyz;
      local_ivox_->getMap(xyz);
      const std::size_t count = xyz.size() / 3;
      const std::size_t displayed = std::min(count, local_map_preview_max_points_);
      marker.points.reserve(displayed);
      for (std::size_t i = 0; i < displayed; ++i)
      {
        const auto index = (i * count / displayed) * 3;
        geometry_msgs::msg::Point point;
        point.x = xyz[index]; point.y = xyz[index + 1]; point.z = xyz[index + 2];
        marker.points.push_back(point);
      }
      marker.scale.x = marker.scale.y = 0.08;
      marker.color.r = 1.0;
      marker.color.g = 0.8;
      marker.color.b = 0.1;
      marker.color.a = 1.0;
      last_local_map_preview_time_ = measures_.lidar.end_time;
    }
    // A latched DELETE also clears the preview for RViz subscribers joining
    // after handoff; stopping publication alone would leave the last map visible.
    local_map_preview_pub_->publish(marker);
  }

  void SuperLIOReLoc::Output()
  {
    publishLocalMapPreview();
    const auto state = kf_->GetNavState();
    data_wrapper_->pub_odom(state);
    if (!g_visual_map)
      return;
    CloudPtr transformed(new PointCloudType);
    pcl::transformPointCloud(*(g_visual_dense ? scan_undistort_full_ : ds_undistort_), *transformed,
                             state.GetSE3().matrix());
    data_wrapper_->pub_cloud_world(transformed, state.timestamp);
  }
} // namespace LI2Sup
