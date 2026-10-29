#pragma once

#include <optional>
#include <cstdint>
#include "loop/SubmapRegistration.hpp"

namespace super_lio_slam
{

  // Poses and all returned source clouds refer to the fixed session frame S.
  RegistrationCloud::Ptr fixedFrameCloud(const SubmapWindow &window, double voxel,
                                         std::size_t max_points);
  struct RelocationSnapshot
  {
    std::uint64_t generation = 0;
    double stamp = 0.0;
    SubmapWindow window;
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
