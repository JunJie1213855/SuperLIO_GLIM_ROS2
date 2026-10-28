#include "lio/ESKF.h"
#include "lio/RelocationRegistration.h"
#include <pcl/common/transforms.h>
#include <iostream>
#include <random>
#include <stdexcept>
using namespace BASIC;
using namespace LI2Sup;
using namespace super_lio_slam;
void require(bool ok,const char* text) { if(!ok) throw std::runtime_error(text); }
int main() {
  try {
    ESKF original, rebased;
    ESKF::Options options;
    V3 bg(.01f,-.02f,.03f),ba(.02f,.03f,-.01f),gravity(0,0,-9.81f);
    SysState initial(10.0,SO3(Eigen::AngleAxisf(.2f,V3::UnitX()).toRotationMatrix()),V3(1,2,3),V3(.3f,-.2f,.1f),bg,ba);
    IMUData imu; imu.secs=10; imu.acc=V3(.1f,0,9.81f); imu.gyr=V3(.01f,.02f,.1f);
    M18 B=M18::Identity(); B(3,0)=.3f; B(8,15)=.2f; B(9,5)=.1f;
    const M18 covariance=.01f*B*B.transpose();
    for(auto* filter:{&original,&rebased}) {
      filter->SetInitialConditions(options,bg,ba,1,gravity);
      filter->InitializePrediction(initial,imu); filter->SetCov(covariance);
    }
    const M3 A=(Eigen::AngleAxisf(.4f,V3::UnitZ())*Eigen::AngleAxisf(.1f,V3::UnitY())).toRotationMatrix();
    const SE3 T(A,V3(4,-3,2));
    const SE3 expression_pose(T.matrix().cast<double>().cast<float>());
    require((expression_pose.R_-T.R_).norm()<1e-7 && (expression_pose.t_-T.t_).norm()<1e-7,
            "SE3 matrix-expression constructor did not initialize rotation/translation");
    M18 J=M18::Identity(); J.block<3,3>(3,3)=A; J.block<3,3>(6,6)=A; J.block<3,3>(15,15)=A;
    rebased.ChangeReferenceFrame(T);
    const auto state=rebased.GetSysState();
    require((state.GetSE3().matrix()-(T*initial.GetSE3()).matrix()).norm()<1e-5,"wrong pose change");
    require((state.v-A*initial.v).norm()<1e-5 && (rebased.GetGravity()-A*gravity).norm()<1e-5,"velocity/gravity not rotated");
    require((state.bg-bg).norm()<1e-7 && (state.ba-ba).norm()<1e-7,"body biases changed");
    require(state.timestamp==initial.timestamp,"reference change modified time");
    require((rebased.GetCov()-J*covariance*J.transpose()).norm()<1e-6,"cross covariance lost");
    rebased.ChangeReferenceFrame(T.inverse());
    require((rebased.GetSE3().matrix()-initial.GetSE3().matrix()).norm()<1e-5,"round trip pose");
    require((rebased.GetCov()-covariance).norm()<1e-6,"round trip covariance");
    rebased.ChangeReferenceFrame(T);
    IMUData next=imu; next.secs=10.01;
    DynamicState forward_a,robot_a,forward_b,robot_b;
    require(original.Predict(next,forward_a,robot_a) && rebased.Predict(next,forward_b,robot_b),"forward prediction unavailable");
    require((forward_b.p-(A*forward_a.p+T.t_)).norm()<1e-5 &&
            (forward_b.v-A*forward_a.v).norm()<1e-5,"forward prediction discontinuity");
    original.SetObsTime(next.secs); rebased.SetObsTime(next.secs);
    original.Predict(next); rebased.Predict(next);
    require((rebased.GetSE3().matrix()-(T*original.GetSE3()).matrix()).norm()<1e-5,"first scan prediction discontinuity");
    const auto before=rebased.GetSE3();
    bool invalid=false;
    try { rebased.ChangeReferenceFrame(SE3(M3(2*M3::Identity()),V3::Zero())); }
    catch(const std::invalid_argument&) { invalid=true; }
    require(invalid && (rebased.GetSE3().matrix()-before.matrix()).norm()==0,"invalid transform mutated state");

    RegistrationCloud::Ptr local(new RegistrationCloud);
    std::mt19937 rng(47); std::uniform_real_distribution<float> random(0,1);
    for(int box=0;box<5;++box) for(int face=0;face<6;++face) for(int k=0;k<500;++k) {
      const Eigen::Vector3f low(box*2.3f-5,box%2*3.1f-3,0),extent(1.5f+box*.2f,2.1f,1.0f+box*.4f);
      Eigen::Vector3f v=low+extent.cwiseProduct(Eigen::Vector3f(random(rng),random(rng),random(rng)));
      v[face/2]=low[face/2]+(face%2)*extent[face/2];
      pcl::PointXYZI p; p.getVector3fMap()=v; p.intensity=1; local->push_back(p);
    }
    RelocationSnapshot task; task.generation=42; task.stamp=2; task.window=SubmapWindow({3,10,10});
    for(int frame=0;frame<3;++frame) {
      Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
      pose.linear()=Eigen::AngleAxisd(.15*frame,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      pose.translation()=Eigen::Vector3d(frame,-.3*frame,0);
      RegistrationCloud::Ptr scan(new RegistrationCloud);
      pcl::transformPointCloud(*local,*scan,pose.inverse().matrix().cast<float>().eval());
      task.window.append({static_cast<std::size_t>(frame),static_cast<double>(frame),pose,scan});
    }
    const auto merged=fixedFrameCloud(task.window,0,0);
    require(merged->size()==3*local->size(),"fixed-S accumulation lost points");
    for(std::size_t k=0;k<merged->size();++k)
      require(((*merged)[k].getVector3fMap()-(*local)[k%local->size()].getVector3fMap()).norm()<1e-4,"moving frames not aligned to S");
#ifdef SUPERLIO_WITH_TEASER
    Eigen::Isometry3d truth=Eigen::Isometry3d::Identity();
    truth.linear()=Eigen::AngleAxisd(.35,Eigen::Vector3d::UnitZ()).toRotationMatrix(); truth.translation()=Eigen::Vector3d(3,-2,0);
    RegistrationCloud::Ptr reference(new RegistrationCloud);
    pcl::transformPointCloud(*local,*reference,truth.matrix().cast<float>().eval());
    task.reference=reference; task.world_from_local_hint=truth.matrix();
    RegistrationSettings settings; settings.voxel=.3; settings.normal_radius=.8; settings.feature_radius=1.6;
    const auto result=registerRelocation(task,settings,80,10);
    std::cout<<"registration "<<result.registration.reason<<" mse="<<result.registration.mse<<std::endl;
    require(result.generation==42 && result.stamp==2,"registration snapshot identity lost");
    require(result.registration.accepted,"motion-compensated TEASER/GICP rejected");
    require((result.registration.transform-truth.matrix()).norm()<.1,"solver returned W<-I instead of W<-S");
    task.local_gravity=Eigen::Vector3d(9.81,0,0);
    const auto tilted=registerRelocation(task,settings,80,10);
    require(!tilted.registration.accepted && tilted.registration.reason=="gravity_mismatch","gravity mismatch accepted");
#endif
    std::cout<<"PASS: ESKF state/covariance/clocks, forward continuity, fixed-S motion accumulation and registration"<<std::endl;
  } catch(const std::exception& e) { std::cerr<<"FAIL: "<<e.what()<<std::endl; return 1; }
}
