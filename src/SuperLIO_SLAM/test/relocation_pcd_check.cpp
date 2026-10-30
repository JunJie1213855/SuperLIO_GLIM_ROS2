// Offline reproduction of the production coarseRegister() path; no ROS/DDS.
#include "loop/SubmapRegistration.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>

using namespace super_lio_slam;
int main(int argc, char **argv)
{
  try
  {
    if (argc != 5)
      throw std::runtime_error("usage: relocation_pcd_check source.pcd target.pcd legacy|legacy_geometry|global report.json");
    const std::string profile = argv[3];
    if (profile != "legacy" && profile != "legacy_geometry" && profile != "global")
      throw std::runtime_error("Unknown profile");
    for (int input = 1; input <= 2; ++input)
      if (std::filesystem::weakly_canonical(argv[input]) == std::filesystem::weakly_canonical(argv[4]))
        throw std::runtime_error("Report path must differ from input PCDs");
    RegistrationSettings s;
    s.coarse_min_inliers = 3;
    s.coarse_min_ratio = .01;
    s.coarse_max_rmse = .4;
    if (profile != "legacy")
    {
      s.voxel = .3; s.normal_radius = .9; s.feature_radius = 1.5;
    }
    if (profile == "global")
    {
      s.feature_matching = "nearest"; s.max_pairs = 2000;
      s.exact_clique = true; s.clique_seconds = 30.; s.threads = 4;
      s.rotation_cost_threshold = 1e-4;
    }
    RegistrationCloud::Ptr raw_source(new RegistrationCloud), raw_target(new RegistrationCloud);
    if (pcl::io::loadPCDFile(argv[1], *raw_source) < 0 || pcl::io::loadPCDFile(argv[2], *raw_target) < 0)
      throw std::runtime_error("Cannot load input PCDs");
    if (raw_source->empty() || raw_target->empty())
      throw std::runtime_error("Empty input PCD");
    auto sample = [&](const RegistrationCloud::Ptr &cloud)
    {
      RegistrationCloud::Ptr result(new RegistrationCloud);
      pcl::VoxelGrid<pcl::PointXYZI> voxel;
      voxel.setInputCloud(cloud); voxel.setLeafSize(s.voxel, s.voxel, s.voxel);
      voxel.filter(*result);
      return result;
    };
    const auto source = sample(raw_source), target = sample(raw_target);
    const auto begin = std::chrono::steady_clock::now();
    const auto result = coarseRegister(source, target, s);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    pcl::KdTreeFLANN<pcl::PointXYZI> tree;
    tree.setInputCloud(raw_target);
    RegistrationCloud aligned;
    pcl::transformPointCloud(*raw_source, aligned, result.transform.cast<float>().eval());
    std::size_t inliers = 0;
    double all_squared = 0, inlier_squared = 0;
    std::vector<int> ids(1); std::vector<float> distances(1);
    for (const auto &p : aligned)
      if (tree.nearestKSearch(p, 1, ids, distances) == 1)
      {
        all_squared += distances[0];
        if (distances[0] <= 1.) { ++inliers; inlier_squared += distances[0]; }
      }
    std::ofstream report(argv[4]);
    report.exceptions(std::ios::failbit | std::ios::badbit);
    auto number = [](double value)
    {
      if (!std::isfinite(value)) return std::string("null");
      std::ostringstream text;
      text << std::setprecision(15) << value;
      return text.str();
    };
    report << std::setprecision(15)
           << "{\n\"profile\":\"" << profile << "\",\n\"accepted\":" << (result.accepted ? "true" : "false")
           << ",\n\"reason\":\"" << result.reason << "\",\n\"source_used\":" << source->size()
           << ",\n\"target_used\":" << target->size() << ",\n\"pairs\":" << result.pairs
           << ",\n\"coarse_inliers\":" << result.coarse_inliers << ",\n\"coarse_rmse_m\":" << number(result.coarse_rmse)
           << ",\n\"coarse_seconds\":" << seconds << ",\n\"coverage_1m\":" << double(inliers) / aligned.size()
           << ",\n\"all_rmse_m\":" << std::sqrt(all_squared / aligned.size())
           << ",\n\"inlier_rmse_m\":" << (inliers ? std::sqrt(inlier_squared / inliers) : -1.)
           << ",\n\"transform\":[";
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        report << (row || col ? "," : "") << result.transform(row, col);
    report << "]\n}\n";
    std::cout << profile << ": " << result.reason << " coarse=" << result.coarse_inliers << "/"
              << result.pairs << " time=" << seconds << " s; report=" << argv[4] << std::endl;
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
