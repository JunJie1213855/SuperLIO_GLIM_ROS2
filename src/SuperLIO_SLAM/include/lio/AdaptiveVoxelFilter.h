#pragma once

#include "basic/Manifold.h"
#include "OctVoxMap/tsl/robin_hood.h"
#include <cstdint>
#include <deque>
#include <optional>

namespace LI2Sup
{
  // 按 GenZ-LIO Algorithm 1 实现扫描采样；地图根体素大小不由本控制器修改。
  struct AdaptiveVoxelConfig
  {
    int window_size = 5;
    int min_points = 1000, max_points = 4000;
    double scale_threshold_m = 30.0;
    int setpoint_exponent = 2;
    double error_sensitivity = 0.1, error_rate_sensitivity = 0.2;
    double p_gain_min = 5e-6, p_gain_max = 5e-5;
    double d_gain_min = 5e-8, d_gain_max = 5e-7;
    double min_leaf_size_m = 0.02, max_leaf_size_m = 1.0;
  };

  struct AdaptiveVoxelStats
  {
    std::size_t input = 0, probe = 0, estimation = 0, mapping = 0, skipped = 0;
    double dt = 0.1, median = 0, scale = 0, target = 0, error = 0;
    double p_gain = 0, d_gain = 0, previous_leaf = 0, leaf = 0, filter_ms = 0;
    bool updated = false, clamped = false, time_reset = false, invalid_time = false;
  };

  struct AdaptiveVoxelOutput
  {
    BASIC::CloudPtr estimation;
    BASIC::CloudPtr mapping;
    AdaptiveVoxelStats stats;
  };

  class AdaptiveVoxelFilter
  {
  public:
    explicit AdaptiveVoxelFilter(const AdaptiveVoxelConfig &config = {}, double initial_leaf = 0.5);
    AdaptiveVoxelOutput process(const BASIC::CloudPtr &deskewed_imu, double scan_end_time,
                                const BASIC::SE3 &imu_from_lidar);
    void reset();

  private:
    // 三个完整的有符号索引共同定义体素身份，避免小体素/负坐标下位打包发生碰撞。
    struct Key
    {
      int64_t x, y, z;
      bool operator==(const Key &other) const { return x == other.x && y == other.y && z == other.z; }
    };
    struct Hash
    {
      std::size_t operator()(const Key &key) const;
    };
    struct Accumulator
    {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      double intensity = 0;
      std::size_t count = 0;
    };
    std::size_t voxelize(const BASIC::PointCloudType &input, double leaf,
                         const Eigen::Matrix3d &lidar_from_imu_rotation,
                         const Eigen::Vector3d &imu_from_lidar_translation,
                         BASIC::PointCloudType &output);

    AdaptiveVoxelConfig config_;
    double initial_leaf_, leaf_, previous_error_ = 0;
    std::optional<double> previous_stamp_;
    std::deque<double> median_window_;
    double median_sum_ = 0;
    robin_hood::unordered_flat_map<Key, Accumulator, Hash> grid_;
    BASIC::PointCloudType probe_;
    std::vector<double> squared_ranges_;
  };
} // namespace LI2Sup
