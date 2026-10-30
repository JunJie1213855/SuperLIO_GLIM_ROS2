#include "loop/SubmapRegistration.hpp"

#include <cmath>
#include <stdexcept>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace super_lio_slam
{
  void SubmapWindow::append(WindowEntry entry)
  {
    // 三个上限任一超出就从队首淘汰旧帧，但始终保留最新锚点帧。
    // 防止异常时间戳
    if (!std::isfinite(entry.stamp) || !entry.odom_pose.matrix().allFinite() || !entry.cloud)
      throw std::invalid_argument("invalid submap entry");
    
    // 非空对比时间，并且计算距离
    if (!entries_.empty())
    {
      if (entry.index <= entries_.back().index || entry.stamp < entries_.back().stamp)
        throw std::invalid_argument("submap index/time moved backwards; restart for a new session");
      // 用平移计算距离
      entry.distance = entries_.back().distance +
                       (entry.odom_pose.translation() - entries_.back().odom_pose.translation()).norm();
    }

    // 加入数据
    entries_.push_back(std::move(entry));
    // 最大帧数，最长时间，最大距离
    while (entries_.size() > 1 &&  // 内部要有数据
           (entries_.size() > static_cast<std::size_t>(std::max(1, limits_.max_frames)) || // 帧数
           (limits_.max_duration > 0 && entries_.back().stamp - entries_.front().stamp > limits_.max_duration) || // 时间差
           (limits_.max_travel > 0 && entries_.back().distance - entries_.front().distance > limits_.max_travel))) // 距离
      entries_.pop_front();
  }
  RegistrationCloud::Ptr SubmapWindow::cloud(double voxel, std::size_t cap) const
  {
    RegistrationCloud::Ptr merged(new RegistrationCloud);
    if (entries_.empty())
      return merged;
    // 以队尾关键帧 A 的 IMU 系为子图坐标系：T_A_i = T_world_A^-1 * T_world_i。
    // 因而队尾自身位姿为 I；源/目标各自锚定后，配准结果就是两个锚点间的相对测量。
    const auto anchor_inverse = entries_.back().odom_pose.inverse();
    // 先按原始 LIO 相对位姿对齐所有成员，再统一降采样，不能直接拼接各自机体系中的点。
    for (const auto &entry : entries_)
    {
      RegistrationCloud transformed;
      pcl::transformPointCloud(*entry.cloud, transformed,
                               (anchor_inverse * entry.odom_pose).matrix().cast<float>().eval());
      *merged += transformed;
    }

    // 降采样
    RegistrationCloud::Ptr filtered(new RegistrationCloud);
    if (voxel > 0)
    {
      pcl::VoxelGrid<pcl::PointXYZI> grid;
      grid.setLeafSize(voxel, voxel, voxel);
      grid.setInputCloud(merged);
      grid.filter(*filtered);
    }
    else
      filtered = merged;
    // 超出点数上限时在整个子图内等间隔取样，限制配准开销而不只保留最后一帧。
    if (cap && filtered->size() > cap)
    {
      RegistrationCloud::Ptr capped(new RegistrationCloud);
      for (std::size_t k = 0; k < cap; ++k)
        capped->push_back((*filtered)[k * filtered->size() / cap]);
      return capped;
    }
    return filtered;
  }
  bool SubmapWindow::sharesFrames(const SubmapWindow &other) const
  {
    // 两个子图共享帧会产生天然重合，可能把同一批点误判为可靠回环，因此需在配准前拒绝。
    for (const auto &a : entries_)
      for (const auto &b : other.entries_)
        if (a.index == b.index)
          return true;
    return false;
  }
  bool validRigidTransform(const Eigen::Matrix4d &t)
  {
    if (!t.allFinite())
      return false;
    const Eigen::Matrix3d r = t.block<3, 3>(0, 0);
    return (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-3 &&
           std::abs(r.determinant() - 1.0) < 1e-3 &&
           (t.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-6;
  }
} // namespace super_lio_slam
