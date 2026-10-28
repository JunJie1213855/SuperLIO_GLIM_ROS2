#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ros/ROSWrapper.h"
#include "lio/super_lio_reloc.h"

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try {
    auto wrapper=std::make_shared<LI2Sup::ROSWrapper>();
    auto lio=std::make_shared<LI2Sup::SuperLIOReLoc>();
    lio->setROSWrapper(wrapper);
    lio->init();
    auto timer=wrapper->create_wall_timer(std::chrono::milliseconds(2),
        [lio] { lio->process(); },wrapper->getSensorCallbackGroup());
    rclcpp::spin(wrapper);
    timer->cancel(); timer.reset();
    lio->printTimeRecord();
    lio.reset(); // Join registration before destroying the ROS wrapper.
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("relocation_node"),"Relocation failed: %s",error.what());
    result=1;
  }
  rclcpp::shutdown();
  return result;
}
