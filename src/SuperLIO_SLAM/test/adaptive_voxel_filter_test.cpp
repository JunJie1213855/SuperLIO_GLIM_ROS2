#include "lio/AdaptiveVoxelFilter.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace BASIC;
using namespace LI2Sup;
namespace
{
void require(bool ok, const std::string &message) { if (!ok) throw std::runtime_error(message); }
void close(double actual, double expected, double tolerance, const std::string &message)
{
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          message + ": " + std::to_string(actual) + " != " + std::to_string(expected));
}
void append(const CloudPtr &cloud, float x, float y, float z, float intensity = 1)
{
  PointType p; p.x = x; p.y = y; p.z = z; p.intensity = intensity; cloud->push_back(p);
}
AdaptiveVoxelConfig frozen()
{
  AdaptiveVoxelConfig config;
  config.p_gain_min = config.p_gain_max = config.d_gain_min = config.d_gain_max = 0;
  return config;
}
CloudPtr fixture(int frame)
{
  CloudPtr cloud(new PointCloudType);
  const float step = frame < 4 ? 0.25f : 0.875f;
  const float shift = frame < 4 ? 1.f : 25.f;
  for (int x = 0; x < 24; ++x) for (int y = 0; y < 24; ++y)
    for (int z = 0; z < 5; ++z) for (int k = 0; k < 2; ++k)
      append(cloud, (x-12)*step+shift+0.0625f+k*0.03125f,
             (y-12)*step+0.0625f, z*step+0.0625f, (x+y+z)%11);
  return cloud;
}

void upstreamFixtures()
{
  // 数值由原版 ScaleAwareVoxelizer.cpp + Downsample.cpp 编译生成，而非本实现自生成。
  // 上游 commit: bf444f7a747ed3da8548b56b49c14f207e8d49bb，初值 .5，时刻 100+i*.125。
  struct Golden { std::size_t probe, estimation, mapping; double leaf, median, scale, target, error; };
  const Golden rows[] = {
    {432,546,3120,0.47894614030588761,2.5499406396277151,2.5499406396277151,1488.3141370401263,-1056.3141370401263},
    {546,588,3120,0.46152308596071229,2.7063293868263707,2.6281350132270429,1502.6033571529085,-956.60335715290853},
    {588,588,3240,0.4447209881733214,2.6696811168424217,2.6419837144321692,1505.1298163953511,-917.1298163953511},
    {588,588,3360,0.42795686162170099,2.5939853055915716,2.6299841122220196,1502.9407676759365,-914.94076767593651},
    {3000,3120,3240,0.45103352020662091,25.970127257401437,7.2980127412579039,2282.0659150130386,717.93408498696135},
    {3120,3000,3240,0.45606050039536505,25.459186279526396,11.87986186923764,2905.5353137403063,214.46468625969374},
    {3000,3840,3840,0.43947071132697302,25.516768383751106,16.441949668622588,3387.2642373727895,-387.26423737278947},
    {3840,2880,3240,0.44231872905383757,28.735067453592308,21.655026935972565,3767.8714152021885,72.128584797811527},
    {2880,3240,3480,0.38751507349119774,25.459186279526396,26.228067130759531,3952.5750747664779,-1072.5750747664779},
    {3240,3120,3360,0.35478855889020472,26.361833947252229,26.306408468729689,3954.524605333761,-714.52460533376097},
  };
  AdaptiveVoxelFilter filter;
  for (int i = 0; i < 10; ++i)
  {
    const auto s = filter.process(fixture(i), 100+i*0.125, SE3()).stats;
    const auto &gold = rows[i];
    const auto frame = " upstream frame " + std::to_string(i);
    require(s.probe == gold.probe && s.estimation == gold.estimation && s.mapping == gold.mapping,
            "cloud count" + frame);
    require(!s.clamped, "rounding was reported as leaf clamping" + frame);
    close(s.leaf, gold.leaf, 1e-6, "leaf" + frame);
    close(s.median, gold.median, 2e-6, "median" + frame);
    close(s.scale, gold.scale, 2e-6, "window" + frame);
    close(s.target, gold.target, 3e-4, "setpoint" + frame);
    close(s.error, gold.error, 3e-4, "error" + frame);
    close(s.dt, i ? 0.125 : 0.1, 1e-10, "sensor dt" + frame);
  }
}

void kernelAndFrames()
{
  CloudPtr cloud(new PointCloudType);
  append(cloud, 0.03125f, 0.03125f, 0.03125f, 1);
  append(cloud, 0.0625f, 0.03125f, 0.03125f, 3);
  append(cloud, 0.1875f, 0.03125f, 0.03125f, 9);
  AdaptiveVoxelFilter filter(frozen(), 0.25);
  const auto out = filter.process(cloud, 1, SE3());
  require(out.mapping->size() == 2 && out.estimation->size() == 1, "dual-resolution kernel");
  close((*out.estimation)[0].x, 0.1171875, 1e-8, "estimation must voxelize mapping centroids");
  close((*out.estimation)[0].intensity, 5.5, 1e-8, "two-stage intensity centroid");

  M3 R; R << 0,-1,0, 1,0,0, 0,0,1;
  const SE3 extrinsic(R, V3(10,20,-3));
  CloudPtr imu(new PointCloudType(*cloud));
  for (auto &p : *imu) p.getVector3fMap() = extrinsic * V3(p.getVector3fMap());
  AdaptiveVoxelFilter transformed(frozen(), 0.25);
  const auto transported = transformed.process(imu, 1, extrinsic);
  require(transported.mapping->size() == 2 && transported.estimation->size() == 1, "grid ignored extrinsic");
  close(((*transported.estimation)[0].getVector3fMap() - extrinsic * V3((*out.estimation)[0].getVector3fMap())).norm(),
        0, 1e-6, "IMU centroid transport");
  close(transported.stats.median, out.stats.median, 1e-8, "range measured in wrong frame");

  CloudPtr boundary(new PointCloudType);
  for (float x : {-0.03125f, 0.f, 0.21875f, 0.25f}) append(boundary, x, 0, 0);
  AdaptiveVoxelFilter floor_filter(frozen(), 0.25);
  const auto edges = floor_filter.process(boundary, 1, SE3());
  require(edges.estimation->size() == 3 && edges.mapping->size() == 4, "floor/negative boundary");

  CloudPtr negative(new PointCloudType);
  append(negative, -20.015625f, 0.015625f, 0.015625f);
  append(negative, -20.015625f, 5.015625f, 2.015625f);
  append(negative, std::numeric_limits<float>::quiet_NaN(), 0, 0);
  append(negative, std::numeric_limits<float>::max(), 0, 0);
  AdaptiveVoxelFilter small(frozen(), 0.02);
  const auto valid = small.process(negative, 1, SE3());
  require(valid.estimation->size() == 2 && valid.stats.skipped == 2, "signed key collision or invalid XYZ");

  CloudPtr even(new PointCloudType);
  append(even, 1,0,0); append(even, 3,0,0);
  AdaptiveVoxelFilter median(frozen(), 0.02);
  close(median.process(even, 1, SE3()).stats.median, 3, 1e-9, "upper median convention");
}

void historiesAndBounds()
{
  AdaptiveVoxelFilter a, b;
  const auto first = a.process(fixture(0), 100, SE3());
  b.process(fixture(0), 100, SE3());
  require(!a.process(CloudPtr(new PointCloudType), 100.05, SE3()).stats.updated, "empty scan advanced history");
  const auto second_a = a.process(fixture(1), 100.125, SE3());
  const auto second_b = b.process(fixture(1), 100.125, SE3());
  close(second_a.stats.leaf, second_b.stats.leaf, 1e-12, "empty scan polluted history");
  const auto rollback = a.process(fixture(0), 2, SE3());
  require(rollback.stats.time_reset, "rollback did not reset");
  close(rollback.stats.leaf, first.stats.leaf, 1e-12, "rollback retained history");
  const auto duplicate = a.process(fixture(0), 2, SE3());
  require(duplicate.stats.time_reset, "duplicate timestamp did not reset");
  const auto invalid = a.process(fixture(0), std::numeric_limits<double>::quiet_NaN(), SE3());
  require(invalid.stats.invalid_time && invalid.estimation->empty(), "nonfinite time accepted");
  close(a.process(fixture(0), 3, SE3()).stats.leaf, first.stats.leaf, 1e-12, "invalid time retained history");
  a.reset();
  close(a.process(fixture(0), 100, SE3()).stats.leaf, first.stats.leaf, 1e-12, "explicit reset");

  auto config = frozen();
  config.p_gain_min = config.p_gain_max = 1;
  config.min_points = config.max_points = 1;
  AdaptiveVoxelFilter high(config, 0.5);
  const auto upper = high.process(fixture(4), 1, SE3());
  require(upper.stats.clamped && upper.stats.leaf == 1.0, "upper bound / positive control direction");
  config.min_points = config.max_points = 100000;
  AdaptiveVoxelFilter low(config, 0.5);
  const auto lower = low.process(fixture(0), 1, SE3());
  require(lower.stats.clamped && lower.stats.leaf == 0.02, "lower bound / negative control direction");

  CloudPtr near(new PointCloudType), far(new PointCloudType);
  append(near, 15,0,0); append(far, 35,0,0);
  AdaptiveVoxelFilter target(frozen(), 0.5);
  close(target.process(near, 1, SE3()).stats.target, 3250, 1e-9, "scale setpoint");
  target.reset();
  close(target.process(far, 1, SE3()).stats.target, 4000, 1e-9, "saturated setpoint");
}

void parameters()
{
  const auto rejected = [](const AdaptiveVoxelConfig &c, double leaf = 0.5) {
    try { AdaptiveVoxelFilter filter(c, leaf); } catch (const std::invalid_argument &) { return true; }
    return false;
  };
  auto c = frozen(); c.window_size = 0; require(rejected(c), "zero window");
  c = frozen(); c.min_points = 5000; require(rejected(c), "inverted points");
  c = frozen(); c.max_leaf_size_m = 0.001; require(rejected(c), "inverted leaf bounds");
  c = frozen(); c.error_sensitivity = 0; require(rejected(c), "zero normalization");
  c = frozen(); c.d_gain_min = -1; require(rejected(c), "negative gain");
  c = frozen(); c.scale_threshold_m = std::numeric_limits<double>::infinity(); require(rejected(c), "infinite scale");
  c = frozen(); c.setpoint_exponent = 1; require(rejected(c), "invalid exponent");
  require(rejected(frozen(), 0), "zero initial leaf");
}
} // namespace

int main()
{
  try
  {
    upstreamFixtures(); kernelAndFrames(); historiesAndBounds(); parameters();
    std::cout << "PASS: 10 upstream scans, centroids/floor/frames, signed keys, PD bounds, sensor time and invalid inputs\n";
  }
  catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
