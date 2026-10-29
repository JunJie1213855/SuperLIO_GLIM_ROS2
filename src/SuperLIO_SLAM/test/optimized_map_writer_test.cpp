#include "loop/OptimizedMapWriter.hpp"
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

using namespace super_lio_slam;
void require(bool value, const char *message)
{
  if (!value) throw std::runtime_error(message);
}
int main()
{
  const auto root = std::filesystem::temp_directory_path() /
                    ("optimized-map-test-" + std::to_string(getpid()));
  std::filesystem::create_directories(root);
  try
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->push_back(pcl::PointXYZI(0, 0, 0, 2));
    cloud->push_back(pcl::PointXYZI(.2, 0, 0, 4));
    cloud->push_back(pcl::PointXYZI(std::numeric_limits<float>::quiet_NaN(), 0, 0, 0));
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(10, 2, -1);
    std::vector<OptimizedMapFrame> frames{{pose, cloud}, {pose, cloud}};
    const auto path = (root / "subdir/map.pcd").string();
    require(writeOptimizedMap(frames, path, 0) == 4, "All retained points must be streamed");
    pcl::PointCloud<pcl::PointXYZI> saved;
    require(pcl::io::loadPCDFile(path, saved) == 0 && saved.size() == 4, "PCD count/header invalid");
    require(std::abs(saved[1].x - 10) < 1e-5 && std::abs(saved[1].y - 2.2) < 1e-5 &&
            std::abs(saved[1].z + 1) < 1e-5 && saved[1].intensity == 4, "Optimized transform/intensity invalid");
    require(writeOptimizedMap(frames, path, 1) == 1, "Voxel fusion must merge overlapping frames");
    require(pcl::io::loadPCDFile(path, saved) == 0 && saved.size() == 1 &&
            std::abs(saved[0].y - 2.1) < 1e-5 && saved[0].intensity == 3, "Voxel centroid invalid");
    bool failed = false;
    try { writeOptimizedMap({}, path, 0); } catch (const std::exception &) { failed = true; }
    require(failed, "Empty export must fail");
    require(pcl::io::loadPCDFile(path, saved) == 0 && saved.size() == 1, "Failed export replaced previous map");
    failed = false;
    try { writeOptimizedMap(frames, path, -1); } catch (const std::exception &) { failed = true; }
    require(failed, "Negative voxel size must fail");
    failed = false;
    try { writeOptimizedMap(frames, root.string(), 0); } catch (const std::exception &) { failed = true; }
    require(failed, "Directory output must fail");
    for (const auto &entry : std::filesystem::directory_iterator(root))
      require(entry.path().filename().string().find(".tmp-") == std::string::npos, "Temporary PCD leaked");
    std::filesystem::remove_all(root);
  }
  catch (...)
  {
    std::filesystem::remove_all(root);
    throw;
  }
}
