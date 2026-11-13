
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

using namespace BASIC;

namespace LI2Sup
{
  // LIO 阅读顺序：process -> 初始化状态机 -> stateProcess。
  // 正常帧依次执行 IMU 传播/去畸变、降采样、点到面迭代更新、地图插入和关键帧发布。
  // 本文件中的 world 表示前端参考系；回环后端另用 map 保存优化结果，不回写前端 ESKF。

  // 用地图中 4/5 个近邻拟合 n^T p + d = 0，归一化后残差的单位为米。
  // 所有近邻都须贴近该平面，避免把跨边缘的点簇当作平面观测。
  inline bool calc_plane_coeff(const int N, const std::array<V3, 5> &points, std::array<double, 4> &abcd)
  {
    Eigen::Vector3d normvec;
    if (N == 5)
    {
      Eigen::Matrix<double, 5, 3> A;
      Eigen::Matrix<double, 5, 1> b;
      for (int j = 0; j < 5; j++)
      {
        A.row(j) = points[j].cast<double>();
        b(j) = -1.0;
      }
      normvec = A.colPivHouseholderQr().solve(b);
    }
    else
    {
      Eigen::Matrix<double, 4, 3> A;
      Eigen::Matrix<double, 4, 1> b;

      for (int j = 0; j < N; j++)
      {
        A.row(j) = points[j].cast<double>();
        b(j) = -1.0;
      }
      normvec = A.colPivHouseholderQr().solve(b);
    }

    double n = normvec.norm();
    if (n < 1e-6f)
      return false;

    abcd[3] = 1.0 / n;
    normvec *= abcd[3];
    abcd[0] = normvec[0];
    abcd[1] = normvec[1];
    abcd[2] = normvec[2];

    for (int i = 0; i < N; ++i)
    {
      const V3 &p = points[i];
      auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
      if (std::abs(dist) > 0.1)
        return false;
    }
    return true;
  }

  inline bool compute_error(
      const std::array<double, 4> &abcd, const V3 &point,
      const float length, scalar &error)
  {
    // 按点的量程筛选点到面残差；这里 length 是距离，返回值只决定该观测是否参与更新。
    error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
    return length > 81 * error * error;
  }

  void SuperLIO::init()
  {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    const auto mode = data_wrapper_->declare_parameter<std::string>(
        "lio.downsample.mode", "fixed", startup_only);
    if (mode != "fixed" && mode != "adaptive")
      throw std::invalid_argument("lio.downsample.mode must be fixed or adaptive");
    if (!std::isfinite(g_voxel_fliter_size) || g_voxel_fliter_size <= 0)
      throw std::invalid_argument("lio.sensor.voxel_fliter_size must be positive and finite");
    adaptive_downsample_ = mode == "adaptive";
    AdaptiveVoxelConfig adaptive_config;
    const auto load = [&]<typename T>(const std::string &name, T &value) {
      value = data_wrapper_->declare_parameter<T>("lio.downsample.adaptive." + name, value, startup_only);
    };
    load("window_size", adaptive_config.window_size);
    load("min_points", adaptive_config.min_points);
    load("max_points", adaptive_config.max_points);
    load("scale_threshold_m", adaptive_config.scale_threshold_m);
    load("setpoint_exponent", adaptive_config.setpoint_exponent);
    load("error_sensitivity", adaptive_config.error_sensitivity);
    load("error_rate_sensitivity", adaptive_config.error_rate_sensitivity);
    load("p_gain_min", adaptive_config.p_gain_min);
    load("p_gain_max", adaptive_config.p_gain_max);
    load("d_gain_min", adaptive_config.d_gain_min);
    load("d_gain_max", adaptive_config.d_gain_max);
    load("min_leaf_size_m", adaptive_config.min_leaf_size_m);
    load("max_leaf_size_m", adaptive_config.max_leaf_size_m);
    if (adaptive_downsample_)
      adaptive_voxel_filter_ = std::make_unique<AdaptiveVoxelFilter>(adaptive_config, g_voxel_fliter_size);
    RCLCPP_INFO(data_wrapper_->get_logger(), "LIO downsample: mode=%s initial_leaf=%.3f m%s",
                mode.c_str(), g_voxel_fliter_size,
                adaptive_downsample_ ? " estimation=d mapping=d/2 (GenZ-LIO)" : " estimation=mapping");

    RCLCPP_INFO(data_wrapper_->get_logger(), "LIO observation: point_to_plane");
    // 有诊断订阅者时才复制残差/雅可比，不修改 ESKF 的门限或权重。
    observation_points_pub_ = data_wrapper_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/observation/points", rclcpp::SensorDataQoS());
    observation_metrics_pub_ = data_wrapper_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/lio/observation/metrics", rclcpp::SensorDataQoS());

    keyframe_enabled_ = data_wrapper_->declare_parameter<bool>(
        "slam.keyframe.enable", true);
    keyframe_distance_ = std::max(0.0, data_wrapper_->declare_parameter<double>(
                                           "slam.keyframe.distance", 1.5));
    const double angle_deg = std::max(0.0, data_wrapper_->declare_parameter<double>(
                                               "slam.keyframe.angle_deg", 10.0));
    keyframe_angle_rad_ = angle_deg * M_PI / 180.0;
    keyframe_max_interval_ = std::max(0.0, data_wrapper_->declare_parameter<double>(
                                               "slam.keyframe.max_interval", 3.0));
    keyframe_min_points_ = std::max<int>(1, data_wrapper_->declare_parameter<int>(
                                                "slam.keyframe.min_points", 100));
    keyframe_voxel_leaf_ = std::max(0.0, data_wrapper_->declare_parameter<double>(
                                             "slam.keyframe.voxel_leaf", 0.5));

    ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
    kf_.reset(new ESKF());
    data_wrapper_->setESKF(kf_);

    scan_undistort_full_.reset(new PointCloudType());
    ds_undistort_.reset(new PointCloudType());
    map_undistort_ = ds_undistort_;
    world_pc_.reset(new PointCloudType());
    ds_world_.reset(new PointCloudType());

    if (g_save_map)
    {
      point_map_.reset(new PointCloudType());
    }

    points_world_v3_.reserve(21000);
    voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

    state_fn_ = &SuperLIO::stateWaitKFInit;

    LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
  }

  void SuperLIO::stateWaitKFInit()
  {
    if (kf_init())
    {
      state_fn_ = &SuperLIO::stateWaitMapInit;
      LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
    }
  }

  void SuperLIO::stateWaitMapInit()
  {
    if (map_init())
    {
      kf_->init_ = true;
      state_fn_ = &SuperLIO::stateProcess;
      LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
    }
  }

  void SuperLIO::process()
  {
    // 只有 IMU 已覆盖本帧扫描结束时刻才推进状态机，不能用未完成同步的数据更新滤波器。
    if (!data_wrapper_->sync_measure(measures_))
    {
      return;
    }
    (this->*state_fn_)();
  }

  bool SuperLIO::kf_init()
  {
    // 启动阶段假设静止/低动态：均值陀螺作为零偏，均值加速度用于重力对齐和尺度归一化。
    // 重力只能约束横滚/俯仰；下面的偏航处理是局部参考系约定，并非绝对航向测量。
    static int imu_cout = 0;
    static V3 mean_gyro = V3::Zero();
    static V3 mean_acce = V3::Zero();

    for (auto &imu : measures_.imu)
    {
      imu_cout++;
      mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
      mean_acce += (imu.acc - mean_acce) / imu_cout;
    }

    /// 100 Hz for 1 second.
    if (imu_cout < 50)
    {
      return false;
    }

    V3 gravity = -mean_acce * g_gravity_norm / mean_acce.norm();
    V3 ref_gravity(0, 0, -g_gravity_norm);
    M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
    V3 n = init_rot.col(0);
    double yaw = atan2(n(1), n(0));

    M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix();

    // init_rot represents the IMU orientation after gravity alignment (level orientation).
    // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
    M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;

    ESKF::Options options;
    options.gyro_var_ = g_imu_ng;
    options.acce_var_ = g_imu_na;
    options.bias_gyro_var_ = g_imu_nbg;
    options.bias_acce_var_ = g_imu_nba;
    options.num_iterations_ = g_kf_max_iterations;
    options.quit_eps_ = g_kf_quit_eps;

    float imu_scale = g_gravity_norm / mean_acce.norm();
    kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
    auto state = kf_->GetSysState();
    state.R = SO3(rot);
    state.p = g_odom_robo.t_; // By default, the robot frame is used as the reference origin.
    state.timestamp = measures_.imu.back().secs;
    kf_->SetX(state);
    sys_init_pose_ = kf_->GetSE3();
    return true;
  }

  bool SuperLIO::map_init()
  {
    // 尚无地图可供匹配时，使用初始位姿与 LiDAR->IMU 外参把启动点云放入前端参考系。
    frame_num_++;

    std::size_t ptsize = measures_.lidar.pc->size();
    points_world_v3_.resize(ptsize);

    const SE3 transform = sys_init_pose_ * g_lidar_imu;

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, ptsize),
        [&](const tbb::blocked_range<size_t> &r)
        {
          for (size_t idx = r.begin(); idx < r.end(); ++idx)
          {
            auto &point_pcl = measures_.lidar.pc->points[idx];
            V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
            points_world_v3_[idx] = transform * point_body;
          }
        });

    ivox_->insert(points_world_v3_);
    kf_->SetLastObsTime(measures_.lidar.end_time);

    // 20 Hz for 1.0 seconds. Integral coverage area > 70%
    if (frame_num_ > 3)
    {
      g_flg_map_init = false;
      return true;
    }
    return false;
  }

  void SuperLIO::stateProcess()
  {
    // 必须先利用旧地图校正本帧，再把校正后的点插入地图，避免本帧与自身形成匹配。
    frame_num_++;
    if (g_time_eva)
    {
      time_record_.Evaluate([this]()
                            { Propagation_Undistort(); }, "[Undistort]");
      time_record_.Evaluate([this]()
                            { DownSample(); }, "[DownSample]");
      time_record_.Evaluate([this]()
                            { Observe(); }, "[Observe]");
      time_record_.Evaluate([this]()
                            { UpdateMap(); }, "[UpdateMap]");
    }
    else
    {
      Propagation_Undistort();
      DownSample();
      Observe();
      UpdateMap();
    }
    Output();
    PublishKeyframe();
    caceData();
  }

  void SuperLIO::PublishKeyframe()
  {
    // 距离、转角或时间任一达到门限即可触发；关键帧频率独立于逐帧 LIO 更新频率。
    if (!keyframe_enabled_ || !scan_undistort_full_ || scan_undistort_full_->empty())
    {
      return;
    }

    const auto state = kf_->GetNavState();
    const BASIC::SE3 pose(state.R.R_, state.p);
    if (have_keyframe_)
    {
      const double distance = (pose.t_ - last_keyframe_pose_.t_).norm();
      const BASIC::M3 delta = last_keyframe_pose_.R_.transpose() * pose.R_;
      const double cos_angle = std::clamp(
          0.5 * (static_cast<double>(delta.trace()) - 1.0), -1.0, 1.0);
      const double angle = std::acos(cos_angle);
      const double elapsed = state.timestamp - last_keyframe_time_;
      if (distance < keyframe_distance_ && angle < keyframe_angle_rad_ &&
          elapsed < keyframe_max_interval_)
      {
        return;
      }
    }

    BASIC::CloudPtr cloud(new BASIC::PointCloudType());
    if (keyframe_voxel_leaf_ > 0.01)
    {
      pcl::VoxelGrid<BASIC::PointType> voxel;
      const float leaf = static_cast<float>(keyframe_voxel_leaf_);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.setInputCloud(scan_undistort_full_);
      voxel.filter(*cloud);
    }
    else
    {
      *cloud = *scan_undistort_full_;
    }
    if (cloud->size() < static_cast<size_t>(keyframe_min_points_))
    {
      return;
    }

    // 点云保留在扫描结束时刻的 IMU 系，位姿单独携带 world<-IMU。
    // 后端才能在 PGO 后重新摆放每帧点云，而不把前端漂移固化进点坐标。
    data_wrapper_->pub_keyframe(cloud, state, next_keyframe_id_++);
    last_keyframe_pose_ = pose;
    last_keyframe_time_ = state.timestamp;
    have_keyframe_ = true;
  }

  void SuperLIO::caceData()
  {
    if (!g_save_map)
      return;
    auto state = kf_->GetNavState();
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
    transformation.block<3, 1>(0, 3) = state.p.cast<float>();

    if (g_if_filter)
    {
      pcl::transformPointCloud(*map_undistort_, *world_pc_, transformation);
    }
    else
    {
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
    }

    static int scan_wait_num = 0;
    if (!world_pc_->empty())
    {
      *point_map_ += *world_pc_;
      scan_wait_num++;
    }

    if (g_pcd_save_interval < 0)
    {
      scan_wait_num = 0;
      return;
    }

    static bool rm_PCD_dir = false;
    if (!rm_PCD_dir)
    {
      rm_PCD_dir = true;
      std::string cmd = "rm -rf " + g_save_map_dir + "/PCD";
      [[maybe_unused]] int res;
      res = system(cmd.c_str());
      cmd = "mkdir -p " + g_save_map_dir + "/PCD";
      res = system(cmd.c_str());
    }

    if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval)
    {
      pcd_index_++;
      std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                           std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *point_map_);
      point_map_->clear();
      scan_wait_num = 0;
    }
  }

  void SuperLIO::ProcessCaceMap()
  {
    namespace fs = std::filesystem;

    std::string pcd_folder = g_save_map_dir + "/PCD";
    std::string output_map_name = g_save_map_dir + "/" + g_map_name;

    LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

    PointCloudType::Ptr merged_map(new PointCloudType());

    int count = 0;
    for (const auto &entry : fs::directory_iterator(pcd_folder))
    {
      if (entry.path().extension() == ".pcd" &&
          entry.path().filename().string().find("scans_") != std::string::npos)
      {
        PointCloudType::Ptr tmp_cloud(new PointCloudType());
        if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0)
        {
          *merged_map += *tmp_cloud;
          count++;
          // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string()
          //           << "   size: " << tmp_cloud->size() << RESET;
        }
        else
        {
          LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
        }
      }
    }

    LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

    PointCloudType filtered_map;

    if (g_if_filter)
    {
      LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
      pcl::VoxelGrid<PointType> voxel_filter;
      voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);

      voxel_filter.setInputCloud(merged_map);
      voxel_filter.filter(filtered_map);
    }
    else
    {
      LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
      filtered_map = *merged_map;
    }

    if (filtered_map.size() > 0)
    {
      filtered_map.width = filtered_map.size();
      filtered_map.height = 1;
      filtered_map.is_dense = false;
    }

    pcl::io::savePCDFileBinary(output_map_name, filtered_map);

    LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
    LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
  }

  void SuperLIO::saveMap()
  {
    if (!g_save_map)
      return;
    if (g_pcd_save_interval > 0)
    {
      LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
      if (point_map_->size() > 0)
      {
        pcd_index_++;
        std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                             std::string(".pcd"));
        LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
        pcl::io::savePCDFileBinary(map_name, *point_map_);
        point_map_->clear();
      }
      LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
      LOG(INFO) << YELLOW << " ---> Process cace map ... " << RESET;
      ProcessCaceMap();
      LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
      return;
    }

    LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
    if (!point_map_->empty())
    {
      std::string map_name = g_save_map_dir + "/" + g_map_name;
      LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
      pcl::VoxelGrid<PointType> voxel_fliter;
      PointCloudType latst_map;
      voxel_fliter.setInputCloud(point_map_);
      voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
      voxel_fliter.filter(latst_map);
      if (latst_map.size() > 0)
      {
        latst_map.width = latst_map.size();
        latst_map.height = 1;
        latst_map.is_dense = false;
      }
      pcl::io::savePCDFileBinary(map_name, latst_map);
      LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
      LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
    }
  }

  inline double get_cpu_time_seconds()
  {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
           usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
  }

  // lidar 点云反向去畸变
  void SuperLIO::Propagation_Undistort()
  {
    // 每个点按自己的采样时间补偿，最终统一到本帧结束时刻的 IMU 系 I_end。
    // 保存中间 IMU 状态供插值；扫描末时刻也是后续点到面更新使用的状态时刻。
    propagate_states_.clear();
    propagate_states_.emplace_back(kf_->GetDynamicState());
    kf_->SetObsTime(measures_.lidar.end_time);
    // std::cout << "size of imu :" << measures_.imu.size() << std::endl;

    // 收集前向传播的状态
    for (auto &imu : measures_.imu)
    {
      kf_->Predict(imu);
      propagate_states_.emplace_back(kf_->GetDynamicState());
    }

    // 多前向传播一个 imu 数据，并且收集该状态，用来插值
    if (!measures_.imu.empty() && kf_->GetTime() < measures_.lidar.end_time)
    {
      auto end_imu = measures_.imu.back();
      end_imu.secs = measures_.lidar.end_time;
      kf_->Predict(end_imu);
      propagate_states_.emplace_back(kf_->GetDynamicState());
    }

    // 外参获取
    static const M3 TLI_R = g_lidar_imu.R_;
    static const V3 TLI_t = g_lidar_imu.t_;
    const SE3 T_end = kf_->GetSE3();
    const M3 R_inv = T_end.R_.transpose();
    const V3 T_end_t = T_end.t_;
    const double start_time = measures_.lidar.start_time;
    auto &raw_pc = measures_.lidar.pc;

    std::size_t ptsize = raw_pc->points.size();
    scan_undistort_full_->resize(ptsize);

    // 并行去畸变
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, ptsize),
        [&](const tbb::blocked_range<size_t> &r)
        {
          M3 R_h, R_t;
          V3 p_h, v_h, acc_t, w_t;
          for (size_t idx = r.begin(); idx < r.end(); ++idx)
          {
            auto &pt_full = scan_undistort_full_->points[idx];
            const auto &pt = raw_pc->points[idx];
            pt_full.intensity = pt.intensity;
            double query_time = start_time + pt.offset_time;
            auto upper = std::upper_bound(propagate_states_.begin(), propagate_states_.end(), query_time,
                                          [](double time, const DynamicState &state)
                                          { return time < state.time; });
            const DynamicState *left;
            const DynamicState *right;
            if (upper == propagate_states_.begin())
            {
              left = right = &propagate_states_.front();
            }
            else if (upper == propagate_states_.end())
            {
              left = right = &propagate_states_.back();
            }
            else
            {
              right = &*upper;
              left = &*std::prev(upper);
            }
            M3 R_i = left->R;
            V3 p_i = left->p;
            const double dt = right->time - left->time;
            if (dt > 1e-9)
            {
              const double tau = std::clamp(query_time - left->time, 0.0, dt);
              R_i = Quat(left->R).slerp(tau / dt, Quat(right->R)).toRotationMatrix();
              p_i = left->p + left->v * tau + .5f * right->a * tau * tau;
            }
            V3 t_ei = p_i - T_end_t;
            V3 raw(pt.x, pt.y, pt.z);
            // p_Iend = R_end^T [R_i (R_IL p_L + t_IL) + p_i - p_end]。
            // 先经外参进入采样时刻的 IMU 系，再经运动补偿进入统一的帧末 IMU 系。
            V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
            pt_full.x = eigen_point[0];
            pt_full.y = eigen_point[1];
            pt_full.z = eigen_point[2];
          }
        });
    // std::cout << "point size : " << scan_undistort_full_->points.size() << "\n";
  }

  void SuperLIO::DownSample()
  {
    if (!adaptive_downsample_)
    {
      voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
      voxel_grid_fliter_.filter(ds_undistort_);
      map_undistort_ = ds_undistort_;
      return;
    }
    auto output = adaptive_voxel_filter_->process(scan_undistort_full_, measures_.lidar.end_time, g_lidar_imu);
    ds_undistort_ = std::move(output.estimation);
    map_undistort_ = std::move(output.mapping);
    adaptive_voxel_stats_ = output.stats;
    const auto &s = adaptive_voxel_stats_;
    if (s.time_reset)
      RCLCPP_WARN(data_wrapper_->get_logger(), "Adaptive voxel history reset: invalid/repeated/backward scan time");
    if (s.updated && (last_adaptive_log_time_ < 0 || s.time_reset ||
                     measures_.lidar.end_time - last_adaptive_log_time_ >= 5.0))
    {
      // 用传感器时间限频；即使未启用 /clock 也能稳定观察控制器工作状态。
      RCLCPP_INFO(data_wrapper_->get_logger(),
                  "Adaptive voxel: input=%zu probe=%zu estimation=%zu mapping=%zu skipped=%zu "
                  "median=%.2f scale=%.2f target=%.0f leaf=%.3f->%.3f m dt=%.4f filter=%.2f ms clamped=%d",
                  s.input, s.probe, s.estimation, s.mapping, s.skipped, s.median, s.scale, s.target,
                  s.previous_leaf, s.leaf, s.dt, s.filter_ms, s.clamped);
      last_adaptive_log_time_ = measures_.lidar.end_time;
    }
  }

  struct ThreadACC
  {
    M6d HTVH = M6d::Zero();
    V6d HTVr = V6d::Zero();
    std::size_t plane_count = 0;
    ThreadACC() : HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
  };

  void SuperLIO::Observe()
  {
    // 观测只直接约束 6 维位姿；速度、零偏和重力通过 ESKF 的交叉协方差间接得到修正。
    size_t ptsize = ds_undistort_->size();

    // 点数预算是软目标，控制器暂态仍可能超过 20000；全部缓存按实际点数扩容。
    points_body_v3_.resize(ptsize);
    point_lengths_.resize(ptsize);
    abcd_vec_.resize(ptsize);
    effect_knn_idxs_.resize(ptsize);
    effect_mask_.assign(ptsize, 0);
    effect_knn_mask_.assign(ptsize, 0);
    plane_candidate_mask_.assign(ptsize, 0);
    const bool capture = observation_points_pub_->get_subscription_count() > 0 ||
                         observation_metrics_pub_->get_subscription_count() > 0;
    // 最近地图点只用于诊断平面支撑距离，不构成距离观测。
    diagnostic_nearest_targets_.resize(capture ? ptsize : 0);
    observation_samples_.resize(capture ? ptsize : 0);
    plane_observation_count_ = 0;
    effect_knn_num_ = ptsize;
    if (ptsize == 0)
    {
      if (capture) PublishObservationDiagnostics(kf_->GetSE3());
      return;
    }

    std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

    for (size_t i = 0; i < ptsize; ++i)
    {
      const auto &point_body_pcl = ds_undistort_->points[i];
      points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
      point_lengths_[i] = points_body_v3_[i].norm();
    }

    ivox_->reset_max_group();
    int iter_num = 0;
    const SE3 prior_pose = kf_->GetSE3();

    kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr)
                       {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();
    // 每次迭代清空实际采用类型，最终保留最后一次线性化真正送进 ESKF 的观测。
    if (capture) std::fill(observation_samples_.begin(), observation_samples_.end(), ObservationSample{});

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if (need_converge && !effect_mask_[idx]) continue;
          effect_mask_[idx] = false;
          // 前几次迭代重新查询；收敛阶段固定候选，但仍用当前位姿重新计算残差和门限。
          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            plane_candidate_mask_[idx] = 0;
            const bool has_plane = top_K.count >= 4 &&
                                   calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
            effect_knn_mask_[idx] = top_K.count >= 4;
            if (has_plane)
              plane_candidate_mask_[idx] = 1;
            if (capture && has_plane) {
              // KNNHeap 是无序堆，points_[0] 不保证最近，必须显式选最小距离。
              int nearest = 0;
              for (int k = 1; k < top_K.count; ++k)
                if (top_K.dist2_[k] < top_K.dist2_[nearest]) nearest = k;
              diagnostic_nearest_targets_[idx] = top_K.points_[nearest];
            }
          }

          if (plane_candidate_mask_[idx] == 1) {
            auto& abcd = abcd_vec_[idx];
            scalar error;
            if (compute_error(abcd, point_world, point_lengths_[idx], error)) {
              V3d normvec(abcd[0], abcd[1], abcd[2]);
              V3d nb = R_transpose * normvec;
              V3d point_body_d = point_body.cast<double>();
              V6d J;
              // r = n^T(Rp+t)+d；右扰动雅可比为 [(p x R^T n)^T, n^T]。
              // J 在此按列向量保存，累加得到 H^T V^-1 H 与 -H^T V^-1 r。
              J.head<3>() = point_body_d.cross(nb);
              J.tail<3>() = normvec;

              // 1000 是当前实现的固定残差信息权重；线程各自累加，最后归并以避免写冲突。
              local_acc.HTVH += J * 1000 * J.transpose();
              local_acc.HTVr -= J * 1000 * error;
              effect_mask_[idx] = true;
              ++local_acc.plane_count;
              if (capture) observation_samples_[idx] = ObservationSample{1, error, 1000.0, J};
            }
          }
          // 无可靠平面或平面残差未通过门限时，直接丢弃该点，不回退到点距离。

        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    plane_observation_count_ = 0;
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
      plane_observation_count_ += local_acc.plane_count;
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++; });

    if (capture) PublishObservationDiagnostics(prior_pose);
    frame_num_++;
  }

  void SuperLIO::PublishObservationDiagnostics(const SE3 &prior_pose)
  {
    const auto state = kf_->GetNavState();
    const SE3 pose = kf_->GetSE3();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "world";
    cloud.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(measures_.lidar.end_time * 1e9)));
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    using Field = sensor_msgs::msg::PointField;
    // 残差与 J 属于更新前的最后线性化；xyz 和 residual_after 用更新后位姿重算，匹配目标保持不变。
    modifier.setPointCloud2Fields(16,
        "x",1,Field::FLOAT32, "y",1,Field::FLOAT32, "z",1,Field::FLOAT32, "rgb",1,Field::FLOAT32,
        "residual",1,Field::FLOAT32, "residual_after",1,Field::FLOAT32,
        "information",1,Field::FLOAT32, "nearest_distance",1,Field::FLOAT32,
        "jrx",1,Field::FLOAT32, "jry",1,Field::FLOAT32, "jrz",1,Field::FLOAT32,
        "jtx",1,Field::FLOAT32, "jty",1,Field::FLOAT32, "jtz",1,Field::FLOAT32,
        "kind",1,Field::UINT32, "source_index",1,Field::UINT32);
    modifier.resize(plane_observation_count_);
    std::size_t output_index = 0;
    for (std::size_t i = 0; i < observation_samples_.size(); ++i)
    {
      const auto &s = observation_samples_[i];
      if (s.kind == 0) continue;
      const V3 world = pose * points_body_v3_[i];
      const double nearest = (world - diagnostic_nearest_targets_[i]).norm();
      const auto &plane = abcd_vec_[i];
      const double after = plane[0]*world.x() + plane[1]*world.y() + plane[2]*world.z() + plane[3];
      const uint32_t rgb = 0x0000ffU; // 当前仅发布蓝色点到面观测源点。
      float packed_rgb;
      std::memcpy(&packed_rgb, &rgb, sizeof(rgb));
      const float values[14] = {world.x(),world.y(),world.z(),packed_rgb,
          static_cast<float>(s.residual),static_cast<float>(after),static_cast<float>(s.information),
          static_cast<float>(nearest),static_cast<float>(s.jacobian[0]),static_cast<float>(s.jacobian[1]),
          static_cast<float>(s.jacobian[2]),static_cast<float>(s.jacobian[3]),
          static_cast<float>(s.jacobian[4]),static_cast<float>(s.jacobian[5])};
      auto *record = cloud.data.data() + output_index * cloud.point_step;
      std::memcpy(record, values, sizeof(values));
      std::memcpy(record + 56, &s.kind, sizeof(s.kind));
      const uint32_t source = static_cast<uint32_t>(i);
      std::memcpy(record + 60, &source, sizeof(source));
      ++output_index;
    }
    cloud.is_dense = true;
    observation_points_pub_->publish(cloud);
    std_msgs::msg::Float64MultiArray metrics;
    metrics.layout.dim.resize(1);
    metrics.layout.dim[0].label = "superlio_observation_v1:stamp,input,planes,points,leaf_m,scale_m,target_points,"
        "x,y,z,correction_m,correction_deg,speed_mps,pose_cov_trace,map_resolution_m,imu_acc_norm,imu_gyro_norm";
    const M3 delta = prior_pose.R_.transpose() * pose.R_;
    const double angle = std::acos(std::clamp(0.5*(static_cast<double>(delta.trace())-1.0),-1.0,1.0))*180.0/M_PI;
    metrics.data = {measures_.lidar.end_time, static_cast<double>(ds_undistort_->size()),
        static_cast<double>(plane_observation_count_),0.0, // v1 协议保留 points 列，恒为 0。
        adaptive_downsample_ ? adaptive_voxel_stats_.leaf : g_voxel_fliter_size,
        adaptive_downsample_ ? adaptive_voxel_stats_.scale : 0.0,
        adaptive_downsample_ ? adaptive_voxel_stats_.target : 0.0,
        pose.t_.x(),pose.t_.y(),pose.t_.z(),(pose.t_-prior_pose.t_).norm(),angle,state.v.norm(),
        kf_->GetCov().topLeftCorner<6,6>().trace(),g_ivox_resolution,
        measures_.imu.empty() ? 0.0 : measures_.imu.back().acc.norm(),
        measures_.imu.empty() ? 0.0 : measures_.imu.back().gyr.norm()};
    metrics.layout.dim[0].size = metrics.layout.dim[0].stride = metrics.data.size();
    observation_metrics_pub_->publish(metrics);
  }

  void SuperLIO::UpdateMap()
  {
    // 使用观测更新后的位姿变换降采样点，地图与 ESKF 始终处于同一个当前参考系。
    const size_t ptsize = map_undistort_->size();
    if (ptsize == 0)
      return;

    last_pose_ = kf_->GetSE3();
    points_world_v3_.resize(ptsize);

    const auto R = last_pose_.R_;
    const auto t = last_pose_.t_;

    for (size_t i = 0; i < ptsize; ++i)
    {
      // 入图点与观测点的质心通常不同，必须独立读取，不能复用观测缓存的索引。
      const auto &pt = (*map_undistort_)[i];
      points_world_v3_[i] = R * V3(pt.x, pt.y, pt.z) + t;
    }

    ivox_->insert(points_world_v3_);
  }

  void SuperLIO::Output()
  {
    auto state = kf_->GetNavState();
    data_wrapper_->pub_odom(state);

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
    transformation.block<3, 1>(0, 3) = state.p.cast<float>();

    // Keep this full, deskewed scan separate from the voxel-filtered points
    // used by Observe() and from the optional keyframe downsampling path.
    CloudPtr dense_world(new PointCloudType());
    if (g_visual_dense && scan_undistort_full_ &&
        !scan_undistort_full_->empty())
    {
      pcl::transformPointCloud(*scan_undistort_full_, *dense_world,
                               transformation);
      data_wrapper_->pub_cloud_world_dense(dense_world, state.timestamp);
    }

    if (g_visual_map)
    {
      static int count = -1;
      count++;
      if (count % g_pub_step != 0)
      {
        return;
      }
      count = 0;
      if (g_visual_dense)
      {
        if (!dense_world->empty())
        {
          data_wrapper_->pub_cloud_world(dense_world, state.timestamp);
        }
      }
      else
      {
        CloudPtr world_pc(new PointCloudType());
        pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
        data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
      }
    }
  }

  void SuperLIO::printTimeRecord()
  {
    if (!g_time_eva)
      return;
    time_record_.PrintAll();
  }

} // namespace END.
