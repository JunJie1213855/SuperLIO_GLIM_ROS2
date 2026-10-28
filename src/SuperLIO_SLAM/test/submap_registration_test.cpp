#include "loop/SubmapRegistration.hpp"
#include <iostream>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <pcl/common/transforms.h>

using namespace super_lio_slam;
void require(bool ok,const char* message) { if (!ok) throw std::runtime_error(message); }
Eigen::Isometry3d pose(double yaw,double x,double y) {
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  t.translation()=Eigen::Vector3d(x,y,0); return t;
}
RegistrationCloud::Ptr one(float x) {
  RegistrationCloud::Ptr cloud(new RegistrationCloud); pcl::PointXYZI p;p.x=x;p.y=0;p.z=0; cloud->push_back(p);return cloud;
}
int main() {
  try {
    SubmapWindow window({2,6,6});
    window.append({0,0,pose(0,0,0),one(1)});
    window.append({1,1,pose(M_PI/2,1,0),one(1)});
    const auto combined=window.cloud(0,0);
    require(combined->size()==2,"lost window points");
    require((*combined)[0].getVector3fMap().norm()<1e-6,"anchor relative transform direction");
    require(((*combined)[1].getVector3fMap()-Eigen::Vector3f(1,0,0)).norm()<1e-6,"current pose must be identity");
    const auto snapshot=window;
    window.append({2,2,pose(0,2,0),one(1)});
    require(window.entries().front().index==1 && snapshot.entries().front().index==0,"snapshot or count trim");
    bool backwards=false;
    try {window.append({3,1,pose(0,0,0),one(1)});} catch(const std::invalid_argument&) {backwards=true;}
    require(backwards && window.entries().size()==2,"timestamp rollback mutated window");
    SubmapWindow travel({10,100,1.5});
    travel.append({0,0,pose(0,0,0),one(1)});travel.append({1,1,pose(0,1,0),one(1)});travel.append({2,2,pose(0,0,0),one(1)});
    require(travel.entries().front().index==1,"travel must be accumulated, not endpoint distance");
    SubmapWindow timed({10,0.5,100});
    timed.append({0,0,pose(0,0,0),one(1)});timed.append({1,1,pose(0,0,0),one(1)});
    require(timed.entries().size()==1,"duration trim");
    require(window.sharesFrames(travel) && !timed.sharesFrames(SubmapWindow{}),"shared members");
    Eigen::Matrix4d invalid=Eigen::Matrix4d::Identity();invalid(0,0)=2;
    require(!validRigidTransform(invalid),"scaled transform accepted");
    RegistrationCloud::Ptr target(new RegistrationCloud);
    std::mt19937 rng(47);std::uniform_real_distribution<float> random(0,1);
    for (int box=0;box<5;++box) for(int face=0;face<6;++face) for(int j=0;j<500;++j) {
      pcl::PointXYZI p;
      Eigen::Vector3f low(box*2.3f-5,box%2*3.1f-3,0), extent(1.5f+box*.2f,2.1f,1.0f+box*.4f);
      Eigen::Vector3f v=low+extent.cwiseProduct(Eigen::Vector3f(random(rng),random(rng),random(rng)));
      v[face/2]=low[face/2]+(face%2)*extent[face/2];p.getVector3fMap()=v;target->push_back(p);
    }
    const auto truth=pose(.35,.7,-.4);
    RegistrationCloud::Ptr source(new RegistrationCloud);
    pcl::transformPointCloud(*target,*source,truth.inverse().matrix().cast<float>().eval());
    // Exercise the real multi-frame builder on two independently posed observations.
    SubmapWindow source_window({2,10,10}), target_window({2,10,10});
    const auto previous=pose(.05,-.2,.1);
    RegistrationCloud::Ptr old_source(new RegistrationCloud), old_target(new RegistrationCloud);
    pcl::transformPointCloud(*source,*old_source,previous.inverse().matrix().cast<float>().eval());
    pcl::transformPointCloud(*target,*old_target,previous.inverse().matrix().cast<float>().eval());
    source_window.append({10,0,previous,old_source});source_window.append({11,1,Eigen::Isometry3d::Identity(),source});
    target_window.append({0,0,previous,old_target});target_window.append({1,1,Eigen::Isometry3d::Identity(),target});
    RegistrationSettings settings;settings.voxel=.3;settings.normal_radius=.8;settings.feature_radius=1.6;
    settings.min_baseline=1;settings.min_inliers=50;settings.quality_radius=.6;settings.max_mse=.05;
    auto src=source_window.cloud(settings.voxel,10000),dst=target_window.cloud(settings.voxel,10000);
    // Direct GICP must recover historical <- current from an imperfect odometry hint,
    // even when TEASER++ is disabled at build time.
    RegistrationResult initial;
    initial.accepted=true;
    initial.transform=pose(.30,.9,-.3).matrix();
    const auto direct=refineRegister(src,dst,source,settings,initial);
    require(direct.accepted,"direct GICP known transform rejected");
    require((direct.transform-truth.matrix()).norm()<.12,"direct GICP transform direction");
    auto boundary_settings=settings;
    boundary_settings.max_mse=direct.mse;
    const auto equal_mse=refineRegister(src,dst,source,boundary_settings,initial);
    require(!equal_mse.accepted && equal_mse.reason=="high_inlier_mse",
            "MSE equal to the strict upper bound accepted");
    boundary_settings.max_mse=std::nextafter(direct.mse,std::numeric_limits<double>::infinity());
    require(refineRegister(src,dst,source,boundary_settings,initial).accepted,
            "MSE immediately below the upper bound rejected");

    // A real cloud with four matching points for each distant outlier gives exactly 80% coverage.
    RegistrationCloud::Ptr eighty_percent(new RegistrationCloud);
    const std::size_t matched=std::min<std::size_t>(src->size(),2000)/4*4;
    for (std::size_t k=0;k<matched;++k) eighty_percent->push_back((*src)[k]);
    for (std::size_t k=0;k<matched/4;++k) {
      pcl::PointXYZI p;p.x=100.0f+k*.1f;p.y=100.0f;p.z=100.0f;
      eighty_percent->push_back(p);
    }
    boundary_settings=settings;
    boundary_settings.min_overlap=.8;
    boundary_settings.min_current_overlap=.8;
    const auto equal_source=refineRegister(eighty_percent,dst,source,boundary_settings,initial);
    require(!equal_source.accepted && equal_source.reason=="low_overlap" && equal_source.overlap==.8,
            "source coverage equal to 80% accepted");
    const auto equal_current=refineRegister(src,dst,eighty_percent,boundary_settings,initial);
    require(!equal_current.accepted && equal_current.reason=="low_overlap" && equal_current.current_overlap==.8,
            "current-frame coverage equal to 80% accepted");
    auto bad_initial=initial;
    bad_initial.transform(0,0)=2;
    const auto invalid_hint=refineRegister(src,dst,source,settings,bad_initial);
    require(!invalid_hint.accepted && invalid_hint.reason=="invalid_initial_transform",
            "invalid GICP initial transform accepted");
    require(!refineRegister(one(0),one(0),one(0),settings,initial).accepted,
            "sparse direct GICP input accepted");
#ifdef SUPERLIO_WITH_TEASER
    auto coarse=coarseRegister(src,dst,settings);
    std::cout<<"coarse "<<coarse.reason<<" pairs="<<coarse.pairs<<" inliers="<<coarse.coarse_inliers<<"\n"<<coarse.transform<<std::endl;
    require(coarse.accepted,"TEASER known transform rejected");
    require(std::isfinite(coarse.coarse_rmse) && coarse.coarse_rmse<=settings.coarse_max_rmse,
            "coarse RMSE not measured");
    auto strict_settings=settings;
    strict_settings.coarse_max_rmse=1e-10;
    const auto rejected_coarse=coarseRegister(src,dst,strict_settings);
    require(!rejected_coarse.accepted && rejected_coarse.reason=="coarse_rmse_exceeded",
            "coarse RMSE gate did not reject inaccurate registration");
    auto fine=refineRegister(src,dst,source,settings,coarse);
    std::cout<<"fine "<<fine.reason<<" mse="<<fine.mse<<" overlap="<<fine.overlap<<std::endl;
    require(fine.accepted,"GICP known transform rejected");
    require((fine.transform-truth.matrix()).norm()<.12,"incorrect full registration transform");
    require(!coarseRegister(one(0),one(0),settings).accepted,"sparse coarse input accepted");
#endif
    settings.min_overlap=1.01;
    require(!refineRegister(src,dst,source,settings,initial).accepted,"quality gate failed to reject");
    settings.min_overlap=.35;
    RegistrationCloud::Ptr plane(new RegistrationCloud);
    for(int k=0;k<600;++k) {pcl::PointXYZI p;p.x=random(rng)*10;p.y=random(rng)*10;p.z=0;plane->push_back(p);}
    initial.transform=Eigen::Matrix4d::Identity();
    const auto degenerate=refineRegister(plane,plane,plane,settings,initial);
    require(!degenerate.accepted && degenerate.reason=="degenerate_geometry","single plane observability gate");
    std::cout<<"PASS: deque geometry, limits, snapshots and registration gates"<<std::endl;
  } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<std::endl;return 1;}
}
