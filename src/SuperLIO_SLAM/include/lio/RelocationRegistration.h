#pragma once

#include <optional>
#include <cstdint>
#include "loop/SubmapRegistration.hpp"

namespace super_lio_slam
{

  // Immutable cumulative LIO map in the fixed session frame S. The scan and
  // S<-I_M pose are captured at the same sensor time, independently of map size.
  struct RelocationSnapshot
  {
    std::uint64_t generation = 0;
    double stamp = 0.0;
    RegistrationCloud::ConstPtr local_map;
    RegistrationCloud::ConstPtr current_cloud; // snapshot scan in I_M
    Eigen::Isometry3d local_from_current = Eigen::Isometry3d::Identity();
    std::size_t frame_count = 0;
    RegistrationCloud::ConstPtr reference;
    std::optional<Eigen::Matrix4d> world_from_local_hint;
    Eigen::Vector3d local_gravity = Eigen::Vector3d(0, 0, -9.81);
  };

  struct RelocationResult
  {
    std::uint64_t generation = 0;
    double stamp = 0.0;
    RegistrationResult registration; // complete W <- S, never W <- I_K
  };

  RelocationResult registerRelocation(const RelocationSnapshot &snapshot,
                                      const RegistrationSettings &settings,
                                      double crop_radius, double max_gravity_angle_deg);
} // namespace super_lio_slam
