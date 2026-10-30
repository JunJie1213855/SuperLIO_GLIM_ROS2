#pragma once

#include <optional>
#include <cstdint>
#include "loop/SubmapRegistration.hpp"

namespace super_lio_slam
{
  // 坐标约定：T_A_B 将 B 系点变换到 A 系；S 是本次启动后固定的局部 LIO 系，W 是先验地图系。
  // Prepare the shared reference used by display, tracking and registration.
  // Keep coordinates in W; remove invalid XYZ, then voxel-filter (0 disables).
  RegistrationCloud::Ptr prepareRelocationReference(const RegistrationCloud::ConstPtr &input,
                                                     double voxel_size);

  // Immutable cumulative LIO map in the fixed session frame S. The scan and
  // S<-I_M pose are captured at the same sensor time, independently of map size.
  struct RelocationSnapshot
  {
    // generation 隔离不同初始位姿先验；旧任务可完成计算，但不能跨代接管当前滤波器。
    std::uint64_t generation = 0;
    double stamp = 0.0;
    RegistrationCloud::ConstPtr local_map; // S 系累积体素地图；与前端正在更新的地图存储分离
    RegistrationCloud::ConstPtr current_cloud; // snapshot scan in I_M
    Eigen::Isometry3d local_from_current = Eigen::Isometry3d::Identity(); // T_S_IM：与快照扫描同一时刻
    std::size_t frame_count = 0;
    RegistrationCloud::ConstPtr reference;
    std::optional<Eigen::Matrix4d> world_from_local_hint;
    Eigen::Vector3d local_gravity = Eigen::Vector3d(0, 0, -9.81);
  };

  struct RelocationResult
  {
    // 返回的是 T_W_S，因此快照 M 配准成功后仍可作用于最新 N 帧：T_W_IN = T_W_S * T_S_IN。
    std::uint64_t generation = 0;
    double stamp = 0.0;
    RegistrationResult registration; // complete W <- S, never W <- I_K
  };

  RelocationResult registerRelocation(const RelocationSnapshot &snapshot,
                                      const RegistrationSettings &settings,
                                      double crop_radius, double max_gravity_angle_deg);
} // namespace super_lio_slam
