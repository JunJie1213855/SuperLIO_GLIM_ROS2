#include "lio/AdaptiveVoxelFilter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace LI2Sup
{
  AdaptiveVoxelFilter::AdaptiveVoxelFilter(const AdaptiveVoxelConfig &config, double initial_leaf)
      : config_(config), initial_leaf_(initial_leaf)
  {
    const auto positive = [](double value)
    { return std::isfinite(value) && value > 0; };
    const auto gains = [](double low, double high)
    {
      return std::isfinite(low) && std::isfinite(high) && low >= 0 && high >= low;
    };
    if (config.window_size < 1 || config.min_points < 1 || config.max_points < config.min_points ||
        !positive(initial_leaf) || !positive(config.scale_threshold_m) ||
        config.setpoint_exponent <= 1 ||
        !positive(config.error_sensitivity) || !positive(config.error_rate_sensitivity) ||
        !gains(config.p_gain_min, config.p_gain_max) || !gains(config.d_gain_min, config.d_gain_max) ||
        !positive(config.min_leaf_size_m) || !positive(config.max_leaf_size_m) ||
        config.max_leaf_size_m < config.min_leaf_size_m ||
        !positive(2.0 / config.min_leaf_size_m))
      throw std::invalid_argument("Invalid adaptive voxel parameters");
    reset();
  }

  void AdaptiveVoxelFilter::reset()
  {
    leaf_ = std::clamp(initial_leaf_, config_.min_leaf_size_m, config_.max_leaf_size_m);
    previous_error_ = 0;
    previous_stamp_.reset();
    median_window_.clear();
    median_sum_ = 0;
  }

  std::size_t AdaptiveVoxelFilter::Hash::operator()(const Key &key) const
  {
    const auto hash = robin_hood::hash<int64_t>{};
    std::size_t seed = hash(key.x);
    seed ^= hash(key.y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= hash(key.z) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  }

  std::size_t AdaptiveVoxelFilter::voxelize(
      const BASIC::PointCloudType &input, double leaf, const Eigen::Matrix3d &rotation,
      const Eigen::Vector3d &translation, BASIC::PointCloudType &output)
  {
    grid_.clear();
    grid_.reserve(input.size());
    std::size_t skipped = 0;
    const double inv_leaf = 1.0 / leaf;
    const double index_limit = std::ldexp(1.0, 63);
    for (const auto &point : input)
    {
      const Eigen::Vector3d p = point.getVector3fMap().cast<double>();
      // 在雷达局部系分箱与统计量程，输出仍为帧末 IMU 系，供原有 ESKF 使用。
      const Eigen::Array3d index = ((rotation * (p - translation)) * inv_leaf).array().floor();
      if (!p.allFinite() || !index.isFinite().all() ||
          (index < -index_limit).any() || (index >= index_limit).any())
      {
        ++skipped;
        continue;
      }
      auto &cell = grid_[Key{static_cast<int64_t>(index.x()), static_cast<int64_t>(index.y()),
                             static_cast<int64_t>(index.z())}];
      cell.sum += p;
      cell.intensity += std::isfinite(point.intensity) ? point.intensity : 0.0;
      ++cell.count;
    }
    output.clear();
    output.reserve(grid_.size());
    for (const auto &entry : grid_)
    {
      const auto &cell = entry.second;
      BASIC::PointType point;
      point.getVector3fMap() = (cell.sum / static_cast<double>(cell.count)).cast<float>();
      point.intensity = static_cast<float>(cell.intensity / cell.count);
      output.push_back(point);
    }
    output.header = input.header;
    output.is_dense = true;
    return skipped;
  }

  AdaptiveVoxelOutput AdaptiveVoxelFilter::process(const BASIC::CloudPtr &input, double stamp,
                                                   const BASIC::SE3 &imu_from_lidar)
  {
    const auto start = std::chrono::steady_clock::now();
    AdaptiveVoxelOutput output{BASIC::CloudPtr(new BASIC::PointCloudType),
                               BASIC::CloudPtr(new BASIC::PointCloudType),
                               {}};
    auto &s = output.stats;
    s.input = input ? input->size() : 0;
    if (!std::isfinite(stamp))
    {
      reset();
      s.invalid_time = s.time_reset = true;
      s.previous_leaf = s.leaf = leaf_;
      return output;
    }
    // 时间回退/重复不能沿用前一会话的微分项；正常首帧 dt=0.1，与上游一致。
    if (previous_stamp_ && (!std::isfinite(stamp - *previous_stamp_) || stamp - *previous_stamp_ <= 1e-9))
    {
      reset();
      s.time_reset = true;
    }
    s.previous_leaf = s.leaf = leaf_;
    if (!input || input->empty())
      return output;
    if (!imu_from_lidar.R_.allFinite() || !imu_from_lidar.t_.allFinite())
      throw std::invalid_argument("Non-finite LiDAR/IMU extrinsic for adaptive voxelization");
    const Eigen::Matrix3d rotation = imu_from_lidar.R_.cast<double>().transpose();
    const Eigen::Vector3d translation = imu_from_lidar.t_.cast<double>();
    s.skipped = voxelize(*input, leaf_, rotation, translation, probe_);
    s.probe = probe_.size();
    // 空扫描不污染场景尺度窗口，也不推进有效扫描的时间与误差历史。
    if (probe_.empty())
      return output;
    squared_ranges_.clear();
    squared_ranges_.reserve(probe_.size());
    for (const auto &point : probe_)
      squared_ranges_.push_back((rotation * (point.getVector3fMap().cast<double>() - translation)).squaredNorm());
    const auto middle = squared_ranges_.begin() + squared_ranges_.size() / 2;
    std::nth_element(squared_ranges_.begin(), middle, squared_ranges_.end());
    s.median = std::sqrt(*middle); // 偶数点使用上中位数。
    median_window_.push_back(s.median);
    median_sum_ += s.median;
    if (median_window_.size() > static_cast<std::size_t>(config_.window_size))
    {
      median_sum_ -= median_window_.front();
      median_window_.pop_front();
    }
    s.scale = median_sum_ / median_window_.size();
    const double phi = std::min(s.scale / config_.scale_threshold_m, 1.0);
    s.target = config_.min_points + (config_.max_points - config_.min_points) *
                                        (1.0 - std::pow(1.0 - phi, config_.setpoint_exponent));
    s.dt = previous_stamp_ ? stamp - *previous_stamp_ : 0.1;
    s.error = static_cast<double>(s.probe) - s.target;
    const double error_rate = (s.error - previous_error_) / s.dt;
    const auto ratio = [](double value, double limit)
    {
      return limit <= 1e-9 ? 0.0 : std::min(std::abs(value), limit) / limit;
    };
    const double psi_p = ratio(s.error, config_.error_sensitivity * s.target);
    const double psi_d = ratio(error_rate, config_.error_rate_sensitivity * s.target / s.dt);
    const double gain_scale = ratio(s.scale, config_.scale_threshold_m);
    s.p_gain = config_.p_gain_min + (config_.p_gain_max - config_.p_gain_min) * std::sqrt(gain_scale * psi_p);
    s.d_gain = config_.d_gain_min + (config_.d_gain_max - config_.d_gain_min) * std::sqrt(gain_scale * psi_d);
    // error=实际点数-目标；点数过多时比例项增大边长。每扫描只执行一次 PD。
    const long double requested = leaf_ + static_cast<long double>(s.p_gain) * s.error +
                                  static_cast<long double>(s.d_gain) * error_rate;
    leaf_ = static_cast<double>(std::clamp(requested, static_cast<long double>(config_.min_leaf_size_m),
                                           static_cast<long double>(config_.max_leaf_size_m)));
    s.clamped = requested < config_.min_leaf_size_m || requested > config_.max_leaf_size_m;
    s.leaf = leaf_;
    previous_error_ = s.error;
    previous_stamp_ = stamp;
    // 必须回到完整扫描，才能在边长减小时恢复细节；二次质心没有点索引对应关系。
    s.skipped = std::max(s.skipped, voxelize(*input, leaf_ * 0.5, rotation, translation, *output.mapping));
    voxelize(*output.mapping, leaf_, rotation, translation, *output.estimation);
    s.mapping = output.mapping->size();
    s.estimation = output.estimation->size();
    s.updated = true;
    s.filter_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return output;
  }
} // namespace LI2Sup
