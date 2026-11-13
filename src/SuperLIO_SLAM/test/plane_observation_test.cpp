#include "lio/super_lio.h"
#include <Eigen/Eigenvalues>
#include <iostream>
#include <cstring>
#include <stdexcept>

using namespace BASIC;
using namespace LI2Sup;
namespace
{
void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }

struct Result
{
  SE3 pose;
  ESKF::COV before, after;
  std::size_t planes;
};
class Harness : public SuperLIO
{
public:
  Result run(const VV3 &map, const VV3 &scan, const V3 &initial)
  {
    ESKF::Options options; options.num_iterations_=5; options.quit_eps_=1e-6;
    kf_->SetInitialConditions(options,V3::Zero(),V3::Zero(),1,V3(0,0,-9.81));
    const M3 identity = M3::Identity();
    kf_->SetX(SysState(100,SE3(identity,initial)));
    kf_->SetObsTime(100);
    kf_->SetCov(.05f*ESKF::COV::Identity());
    ivox_->insert(map);
    ds_undistort_->clear();
    for(const V3 &p:scan) { PointType pt;pt.getVector3fMap()=p;pt.intensity=1;ds_undistort_->push_back(pt); }
    measures_.lidar.end_time=100;
    const auto before=kf_->GetCov();
    Observe();
    return {kf_->GetSE3(),before,kf_->GetCov(),plane_observation_count_};
  }
};
Result run(const VV3 &map, const VV3 &scan, const V3 &initial)
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.parameter_overrides({
    {"lio.sensor.voxel_fliter_size",.25},
    {"lio.hash_map.vox_resolution",.5}, {"lio.hash_map.hash_capacity",10000},
    {"lio.output.pub_step",1}, {"slam.keyframe.enable",false},
    {"lio.extrinsic.lidar_imu",std::vector<double>{0,0,0,1,0,0,0,1,0,0,0,1}},
  });
  auto wrapper=std::make_shared<ROSWrapper>(options);
  Harness lio;lio.setROSWrapper(wrapper);lio.init();
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  std_msgs::msg::Float64MultiArray::ConstSharedPtr metrics;
  auto cloud_sub = wrapper->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lio/observation/points", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloud = msg; });
  auto metrics_sub = wrapper->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/lio/observation/metrics", rclcpp::SensorDataQoS(),
      [&](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) { metrics = msg; });
  auto result=lio.run(map,scan,initial);
  rclcpp::spin_some(wrapper);
  require(cloud && metrics, "observation diagnostics were not published");
  require(cloud->width * cloud->height == result.planes, "diagnostic plane count differs from ESKF");
  require(metrics->data.size() == 17 && metrics->data[2] == result.planes && metrics->data[3] == 0,
          "diagnostic protocol reports removed point observations");
  for (std::size_t i = 0; i < cloud->width * cloud->height; ++i) {
    uint32_t kind, rgb;
    std::memcpy(&kind, cloud->data.data() + i * cloud->point_step + 56, sizeof(kind));
    std::memcpy(&rgb, cloud->data.data() + i * cloud->point_step + 12, sizeof(rgb));
    require(kind == 1 && rgb == 0x0000ffU, "diagnostics contain non-plane or non-blue points");
  }
  require(result.after.allFinite() && result.pose.matrix().allFinite(), "ESKF became non-finite");
  require((result.after-result.after.transpose()).norm()<1e-6, "covariance lost symmetry");
  Eigen::SelfAdjointEigenSolver<ESKF::COV> eig(result.after);
  require(eig.info()==Eigen::Success && eig.eigenvalues().minCoeff()>-1e-6, "covariance lost positive semidefiniteness");
  return result;
}

void productionTest()
{
  // 分散的孤立体素各自仅存一个地图点：原实现 (<4 近邻) 无法产生任何平面。
  VV3 sparse;
  for(int x=-1;x<=1;++x) for(int y=-1;y<=1;++y)
    sparse.emplace_back(4*x+.3f,4*y+.3f,1.3f);
  const V3 offset(.08f,-.06f,.04f);
  const auto plane=run(sparse,sparse,offset);
  require(plane.planes==0, "sparse test unexpectedly fitted a plane");
  require((plane.pose.t_-offset).norm()<1e-6, "isolated nearest points incorrectly moved state");
  require((plane.after-plane.before).norm()<1e-6, "isolated points incorrectly added observation information");
  const auto empty=run({},sparse,offset);
  require(empty.planes==0 && (empty.pose.t_-offset).norm()<1e-6,"empty map produced observations");

  VV3 surface;
  for(int x=-8;x<=8;++x) for(int y=-8;y<=8;++y)
    surface.emplace_back(x*.18f+.03f,y*.18f+.04f,2.0f);
  const auto ordinary=run(surface,surface,V3(0,0,.03f));
  require(ordinary.planes>0, "valid surface produced no plane observations");
  require(std::abs(ordinary.pose.t_.z())<.01f, "plane observation did not correct normal displacement");
  require(ordinary.after.trace()<ordinary.before.trace(), "plane information did not update covariance");

}
} // namespace
int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);int code=0;
  try {productionTest();std::cout<<"PASS: sparse points rejected, plane pose correction, covariance and blue-only diagnostics\n";}
  catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';code=1;}
  rclcpp::shutdown();return code;
}
