#include "mytest/PX4Ctrlfsm.h"
#include "mytest/PX4Ctrlparam.h"
#include <ros/ros.h>
#include <signal.h>
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "myctrl");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~"); //使用私有命名空间

  mytest_px4ctrlparam::Parameter_t param;
  param.config_from_ros_handle(nh_);
}