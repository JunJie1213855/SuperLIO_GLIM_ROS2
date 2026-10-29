#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace super_lio_slam
{
  // 优化后的帧
  struct OptimizedMapFrame
  {
    Eigen::Isometry3d pose;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
  };

  // 写入所有的帧为点云
  // Writes all retained points in map coordinates. voxel_size=0 streams directly;
  // otherwise memory grows with occupied voxels, not the sum of input clouds.
  std::size_t writeOptimizedMap(const std::vector<OptimizedMapFrame> &frames,
                                const std::string &path, double voxel_size);
}
