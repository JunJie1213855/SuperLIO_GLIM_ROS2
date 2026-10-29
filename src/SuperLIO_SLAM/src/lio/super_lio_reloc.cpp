#include "lio/super_lio_reloc.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
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
#ifndef SUPERLIO_WITH_TEASER
    throw std::runtime_error("Motion-compensated relocation requires SUPERLIO_WITH_TEASER=ON");
#endif
    SuperLIO::init();
    keyframe_enabled_ = false; // Never mix local-S keyframes into the world-frame loop graph.
    data_wrapper_->enableRelocationFrames();
    auto &node = *data_wrapper_;
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
    SubmapLimits limits;
    limits.max_frames = integer("relocation.window_max_frames", 10, 2, 100);
    limits.max_duration = positive("relocation.window_max_duration_s", 3.0);
    limits.max_travel = positive("relocation.window_max_travel_m", 5.0);
    window_ = SubmapWindow(limits);
    min_frames_ = integer("relocation.window_min_frames", 5, 2, limits.max_frames);
    imu_samples_ = integer("relocation.imu_init_samples", 50, 20, 10000);
    init_duration_ = positive("relocation.imu_init_duration_s", .3);
    max_gyro_ = positive("relocation.imu_max_gyro_rad_s", .15);
    max_gyro_std_ = positive("relocation.imu_max_gyro_std", .02);
    max_accel_std_ = positive("relocation.imu_max_accel_std", .15);
    min_plane_points_ = integer("relocation.local_min_plane_points", 30, 6, 20000);
    keyframe_distance_ = positive("relocation.keyframe_distance_m", .2);
    keyframe_angle_ = positive("relocation.keyframe_angle_deg", 5.0);
    keyframe_interval_ = positive("relocation.keyframe_max_interval_s", .3);
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
    s.voxel = positive("registration_voxel_size_m", .5);
    s.min_points = integer("registration_min_points", 100, 30, 100000);
    s.max_source = integer("registration_max_source_points", 10000, 100, 100000);
    s.max_target = integer("registration_max_target_points", 50000, 100, 200000);
    s.normal_radius = positive("fpfh_normal_radius_m", 1.0);
    s.feature_radius = positive("fpfh_feature_radius_m", 2.5);
    s.feature_ratio = positive("fpfh_distance_ratio_threshold", .9);
    s.hint_radius = positive("fpfh_pose_hint_radius_m", 3.0);
    s.min_pairs = integer("correspondence_min_unique", 6, 3, 500);
    s.max_pairs = integer("correspondence_max_unique", 200, 3, 500);
    s.min_baseline = positive("correspondence_min_baseline_m", 2.0);
    s.noise_bound = positive("teaser_noise_bound_m", .3);
    s.rotation_iterations = integer("teaser_rotation_max_iterations", 100, 1, 1000);
    s.clique_seconds = positive("teaser_max_clique_time_limit_s", .05);
    s.threads = integer("teaser_max_clique_num_threads", 2, 1, 16);
    s.coarse_radius = positive("coarse_residual_threshold_m", .6);
    s.coarse_min_inliers = integer("coarse_min_inliers", 6, 3, 500);
    s.coarse_min_ratio = positive("coarse_min_inlier_ratio", .3);
    s.coarse_max_rmse = positive("coarse_max_inlier_rmse_m", .3);
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
    setStatus("LOAD_REFERENCE_MAP");
    std::string map_path = node.declare_parameter<std::string>("relocation.map_path", "");
    if (map_path.empty())
      map_path = g_save_map_dir + "/" + g_map_name;
    reference_cloud_.reset(new PointCloudType);
    if (pcl::io::loadPCDFile<PointType>(map_path, *reference_cloud_) < 0)
      throw std::runtime_error("Cannot load relocation reference map: " + map_path);
    CloudPtr finite(new PointCloudType);
    VV3 points;
    for (const auto &p : *reference_cloud_)
      if (p.getVector3fMap().allFinite())
      {
        finite->push_back(p);
        points.push_back(p.getVector3fMap());
      }
    reference_cloud_ = finite;
    if (points.size() < static_cast<std::size_t>(s.min_points))
      throw std::runtime_error("Relocation reference map is empty/sparse");
    reference_ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
    reference_ivox_->insert(points);
    reference_tree_.setInputCloud(reference_cloud_);
    local_ivox_ = ivox_;
    data_wrapper_->set_global_map(reference_cloud_);

    // 设置初始化位姿猜测回调函数
    data_wrapper_->setRelocationInitialPoseCallback([this](const auto &msg)
                                                    { receivePrior(msg); });
    setStatus("WAITING_FOR_STATIONARY_IMU");
    registration_thread_ = std::thread(&SuperLIOReLoc::registrationWorker, this);
    RCLCPP_INFO(node.get_logger(), "Relocation reference loaded: %zu points, %s", points.size(), map_path.c_str());
  }

  bool SuperLIOReLoc::initializeLocal()
  {
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
    // 方差
    const float acc_std = (acc_square / n - mean_acc.cwiseProduct(mean_acc)).cwiseMax(0).sum();
    const float gyro_std = (gyro_square / n - mean_gyro.cwiseProduct(mean_gyro)).cwiseMax(0).sum();
    if (std::sqrt(acc_std) > max_accel_std_ || std::sqrt(gyro_std) > max_gyro_std_ || mean_acc.norm() < 1e-3)
    {
      initialization_imu_.clear();
      return false;
    }
    // 初始化旋转估计
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
    rememberFrame();
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

  void SuperLIOReLoc::rememberFrame()
  {
    const auto pose = kf_->GetSE3();
    if (!window_.entries().empty())
    {
      const auto &last = window_.entries().back();
      const auto delta = last.odom_pose.inverse() * Eigen::Isometry3d(pose.matrix().cast<double>());
      if (delta.translation().norm() < keyframe_distance_ &&
          Eigen::AngleAxisd(delta.rotation()).angle() * 180 / M_PI < keyframe_angle_ &&
          measures_.lidar.end_time - last.stamp < keyframe_interval_)
        return;
    }
    CloudPtr cloud(new PointCloudType(*scan_undistort_full_));
    window_.append({next_frame_++, measures_.lidar.end_time, Eigen::Isometry3d(pose.matrix().cast<double>()), cloud});
  }

  void SuperLIOReLoc::enqueueRegistration()
  {
    if (window_.entries().size() < static_cast<std::size_t>(min_frames_) ||
        (last_task_time_ >= 0 && measures_.lidar.end_time - last_task_time_ < registration_interval_))
      return;
    RelocationSnapshot task;
    task.generation = generation_;
    task.stamp = measures_.lidar.end_time;
    task.window = window_;
    task.reference = reference_cloud_;
    task.local_gravity = kf_->GetGravity().cast<double>();
    if (have_prior_)
      task.world_from_local_hint = (prior_pose_ * prior_local_pose_.inverse()).matrix().cast<double>();
    {
      std::lock_guard<std::mutex> lock(registration_mutex_);
      pending_task_ = std::move(task); // Latest pending window wins; running solver is not interrupted.
    }
    last_task_time_ = measures_.lidar.end_time;
    registration_cv_.notify_one();
  }

  // 配准重定位线程
  void SuperLIOReLoc::registrationWorker()
  {
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
                  "Relocation registration %s: generation=%lu stamp=%.6f coarse=%zu/%zu rmse=%.4f "
                  "mse=%.4f overlap=%.3f current_overlap=%.3f stage=%s",
                  decisionText(r.accepted, r.accepted ? "accepted" : "rejected").c_str(),
                  static_cast<unsigned long>(result.generation), result.stamp, r.coarse_inliers, r.pairs,
                  r.coarse_rmse, r.mse, r.overlap, r.current_overlap, decisionText(r.accepted, r.reason).c_str());
      std::lock_guard<std::mutex> lock(registration_mutex_);
      if (!stopping_)
        pending_result_ = std::move(result);
    }
  }

  // 提交配准任务
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

    // 配准成功，就直接改变参考帧，从当前局部坐标系转换到全局坐标系上，同时更新状态和协方差
    kf_->ChangeReferenceFrame(world_from_local, position_variance_, rotation_variance_);
    sys_init_pose_ = world_from_local * sys_init_pose_;
    last_pose_ = kf_->GetSE3();
    propagate_states_.clear();
    points_world_v3_.clear();
    world_pc_->clear();
    ds_world_->clear();
    ivox_ = reference_ivox_;
    local_ivox_.reset();

    // 设置相关信号
    data_wrapper_->commitRelocationFrame(world_from_local);
    stage_ = Stage::World;
    ++generation_;
    stopping_ = true;
    registration_cv_.notify_all();
    window_ = SubmapWindow();
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
      auto best = window_.entries().end();
      double gap = .25;
      for (auto it = window_.entries().begin(); it != window_.entries().end(); ++it)
        if (std::abs(it->stamp - stamp) <= gap)
        {
          best = it;
          gap = std::abs(it->stamp - stamp);
        }
      if (best == window_.entries().end())
      {
        RCLCPP_WARN(data_wrapper_->get_logger(), "Initial pose timestamp has no matching local keyframe (tolerance 0.25 s)");
        return;
      }
      local_pose = SE3(best->odom_pose.matrix().cast<scalar>());
    }
    // 四元数归一化
    rotation.normalize();
    // 先验位姿
    prior_pose_ = SE3(rotation, V3(p.x, p.y, p.z));
    prior_local_pose_ = local_pose;
    have_prior_ = true;
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

    // 初始化完后开始局部建图
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
        rememberFrame();
        if (!commitRegistration()) // 如果配准失败
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

  void SuperLIOReLoc::Output()
  {
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
