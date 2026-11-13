#include "lio/super_lio.h"
#include <iostream>
#include <stdexcept>

using namespace BASIC;
using namespace LI2Sup;
namespace
{
void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }

// 通过受保护接口检查真实前端数据流；不引入 friend，也不复制生产采样逻辑。
class Harness : public SuperLIO
{
public:
  using SuperLIO::DownSample;
  using SuperLIO::Observe;
  using SuperLIO::UpdateMap;
  void scan(const CloudPtr &cloud) { scan_undistort_full_ = cloud; measures_.lidar.end_time = 100.0; }
  CloudPtr estimation() const { return ds_undistort_; }
  CloudPtr mapping() const { return map_undistort_; }
  void setPose(const SE3 &pose)
  {
    ESKF::Options options;
    options.num_iterations_ = 4;
    kf_->SetInitialConditions(options, V3::Zero(), V3::Zero(), 1, V3(0, 0, -9.81));
    kf_->SetX(SysState(100.0, pose));
  }
  void checkObservationCache(std::size_t count)
  {
    require(effect_mask_.size() == count && effect_knn_mask_.size() == count &&
            abcd_vec_.size() == count && effect_knn_idxs_.size() == count &&
            point_lengths_.size() == count, "observation cache size mismatch");
  }
  void checkMap(const SE3 &pose)
  {
    require(points_world_v3_.size() == map_undistort_->size(), "mapping used estimation count");
    for (std::size_t i = 0; i < map_undistort_->size(); ++i)
    {
      const V3 expected = pose * V3((*map_undistort_)[i].getVector3fMap());
      require((points_world_v3_[i] - expected).norm() < 1e-5, "mapping used wrong points or pose");
    }
  }
};

void run(const std::string &mode, const CloudPtr &scan)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"lio.downsample.mode", mode}, {"lio.sensor.voxel_fliter_size", 0.02},
    {"lio.downsample.adaptive.p_gain_min", 0.0}, {"lio.downsample.adaptive.p_gain_max", 0.0},
    {"lio.downsample.adaptive.d_gain_min", 0.0}, {"lio.downsample.adaptive.d_gain_max", 0.0},
    {"lio.extrinsic.lidar_imu", std::vector<double>{0,0,0,1,0,0,0,1,0,0,0,1}},
    {"lio.hash_map.hash_capacity", 50000}, {"lio.hash_map.vox_resolution", 0.5},
    {"lio.output.pub_step", 1}, {"slam.keyframe.enable", false},
  });
  auto wrapper = std::make_shared<ROSWrapper>(options);
  Harness lio;
  lio.setROSWrapper(wrapper);
  lio.init();
  lio.scan(scan);
  lio.DownSample();
  if (mode == "fixed")
  {
    VoxelGridClosest<PointType> legacy;
    legacy.setLeafSize(0.02f);
    legacy.setInputCloud(scan);
    CloudPtr reference(new PointCloudType);
    legacy.filter(reference);
    require(lio.mapping() == lio.estimation(), "fixed mode stopped sharing its cloud");
    require(reference->size() == lio.estimation()->size(), "fixed baseline count changed");
    for (std::size_t i = 0; i < reference->size(); ++i)
      require(((*reference)[i].getVector4fMap() == (*lio.estimation())[i].getVector4fMap()) &&
              (*reference)[i].intensity == (*lio.estimation())[i].intensity, "fixed baseline points changed");
  }
  else
  {
    require(lio.estimation()->size() > 20000, "test did not exercise >20000 estimation points");
    require(lio.mapping()->size() > lio.estimation()->size(), "adaptive mapping was not denser");
  }
  const SE3 pose(Eigen::AngleAxisf(0.3f, V3::UnitZ()).toRotationMatrix(), V3(3, 2, 1));
  lio.setPose(pose);
  lio.Observe(); // 空地图使匹配失败，但仍遍历全部工作缓存，能暴露原来的越界。
  lio.checkObservationCache(lio.estimation()->size());
  lio.UpdateMap();
  lio.checkMap(pose);
  lio.scan(CloudPtr(new PointCloudType));
  lio.DownSample();
  lio.Observe();
  lio.UpdateMap();
  lio.checkObservationCache(0);
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    CloudPtr cloud(new PointCloudType);
    for (int x = 0; x < 30; ++x)
      for (int y = 0; y < 30; ++y)
        for (int z = 0; z < 25; ++z)
          for (float offset : {0.002f, 0.014f})
          {
            PointType p;
            p.x = x * 0.08f + offset; p.y = y * 0.08f + 0.002f; p.z = z * 0.08f + 0.002f;
            p.intensity = 1;
            cloud->push_back(p);
          }
    run("fixed", cloud);
    run("adaptive", cloud);
    bool rejected = false;
    try { run("genz_fixed", cloud); } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "unsupported third mode was accepted");
    std::cout << "PASS: fixed baseline, >20000 dynamic caches, independent mapping and updated pose, empty scan\n";
  }
  catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; result = 1; }
  rclcpp::shutdown();
  return result;
}
