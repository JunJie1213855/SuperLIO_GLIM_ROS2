#include "lio/RelocationRegistration.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace super_lio_slam
{
  namespace
  {
    RegistrationCloud::Ptr sample(const RegistrationCloud::ConstPtr &input, double voxel,
                                  std::size_t cap)
    {
      RegistrationCloud::Ptr output(new RegistrationCloud);
      if (!input || input->empty())
        return output;
      if (voxel > 0)
      {
        pcl::VoxelGrid<pcl::PointXYZI> grid;
        grid.setInputCloud(input);
        grid.setLeafSize(voxel, voxel, voxel);
        grid.filter(*output);
      }
      else
      {
        *output = *input;
      }
      if (cap && output->size() > cap)
      {
        RegistrationCloud::Ptr capped(new RegistrationCloud);
        for (std::size_t i = 0; i < cap; ++i)
          capped->push_back((*output)[i * output->size() / cap]);
        return capped;
      }
      return output;
    }
  }

  RegistrationCloud::Ptr prepareRelocationReference(const RegistrationCloud::ConstPtr &input,
                                                     double voxel_size)
  {
    if (!std::isfinite(voxel_size) || voxel_size < 0)
      throw std::invalid_argument("relocation.reference_map_voxel_size_m must be finite and nonnegative");
    RegistrationCloud::Ptr finite(new RegistrationCloud);
    if (input)
    {
      finite->header = input->header;
      finite->sensor_origin_ = input->sensor_origin_;
      finite->sensor_orientation_ = input->sensor_orientation_;
      finite->reserve(input->size());
      // Do not trust is_dense in imported PCD files.
      for (const auto &point : *input)
        if (point.getVector3fMap().allFinite())
          finite->push_back(point);
    }
    finite->is_dense = true;
    return sample(finite, voxel_size, 0);
  }

  RelocationResult registerRelocation(const RelocationSnapshot &task,
                                      const RegistrationSettings &settings,
                                      double crop_radius, double max_gravity_angle_deg)
  {
    RelocationResult output;
    output.generation = task.generation;
    output.stamp = task.stamp;
    if (!task.local_map || task.local_map->empty() ||
        !task.current_cloud || task.current_cloud->empty() ||
        !task.local_from_current.matrix().allFinite() ||
        !task.reference || task.reference->empty())
    {
      output.registration.reason = "empty_relocation_snapshot";
      return output;
    }
    // The source already lives in S. Do not re-anchor it at I_M or transform it
    // by the snapshot pose again; the solver must return W<-S.
    const auto source = sample(task.local_map, settings.voxel, settings.max_source);
    RegistrationCloud::Ptr current(new RegistrationCloud);
    pcl::transformPointCloud(*task.current_cloud, *current, task.local_from_current.matrix().cast<float>().eval());
    current = sample(current, settings.voxel, settings.max_source);
    RegistrationCloud::ConstPtr target_input = task.reference;
    if (task.world_from_local_hint && crop_radius > 0)
    {
      RegistrationCloud::Ptr cropped(new RegistrationCloud);
      const Eigen::Vector3d centre = ((*task.world_from_local_hint) * task.local_from_current.matrix()).block<3, 1>(0, 3);
      for (const auto &p : *task.reference)
        if ((p.getVector3fMap().cast<double>() - centre).squaredNorm() <= crop_radius * crop_radius)
          cropped->push_back(p);
      target_input = cropped;
    }
    RegistrationCloud::Ptr target;
    RegistrationCloud::Ptr hinted_target;
    if (task.world_from_local_hint)
    {
      // Use one approximate frame for feature viewpoints and voxel grids. A remote
      // world origin otherwise flips normals compared with the local sensor origin.
      RegistrationCloud::Ptr local_target(new RegistrationCloud);
      pcl::transformPointCloud(*target_input, *local_target,
                               task.world_from_local_hint->inverse().cast<float>().eval());
      hinted_target = sample(local_target, settings.voxel, settings.max_target);
      target.reset(new RegistrationCloud);
      pcl::transformPointCloud(*hinted_target, *target, task.world_from_local_hint->cast<float>().eval());
    }
    else
    {
      target = sample(target_input, settings.voxel, settings.max_target);
    }
    auto attempt = [&](const Eigen::Matrix4d *hint)
    {
      RegistrationResult result;
      if (hint)
      {
        const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
        result = coarseRegister(source, hinted_target, settings, &identity);
        if (result.accepted)
          result.transform = (*hint * result.transform).eval();
      }
      else
      {
        // teaser++ 粗配准
        result = coarseRegister(source, target, settings);
      }

      // gicp 精配准
      if (result.accepted)
        result = refineRegister(source, target, current, settings, result);
      return result;
    };
    
    output.registration = attempt(task.world_from_local_hint ? &*task.world_from_local_hint : nullptr);
    if (!output.registration.accepted && task.world_from_local_hint)
      output.registration = attempt(nullptr);
    if (output.registration.accepted)
    {
      const Eigen::Vector3d gravity = output.registration.transform.block<3, 3>(0, 0) * task.local_gravity;
      const double angle = std::acos(std::clamp(gravity.normalized().dot(-Eigen::Vector3d::UnitZ()), -1.0, 1.0)) * 180.0 / M_PI;
      if (!std::isfinite(angle) || angle > max_gravity_angle_deg)
      {
        output.registration.accepted = false;
        output.registration.reason = "gravity_mismatch";
      }
    }
    return output;
  }
} // namespace super_lio_slam
