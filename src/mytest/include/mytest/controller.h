#ifndef MYTEST__CONTROLLER_H
#define MYTEST__CONTROLLER_H
#include "mytest/ROScallback.h"
#include <Eigen/Dense>
namespace mytest_contorller {
struct Desired_State_t {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;
  double head_rate;

  Desired_State_t(){};
  Desired_State_t(Odom_Data_t &odom)
      : p(odom.p), v(Eigen::Vector3d::Zero()), a(Eigen::Vector3d::Zero()),
        j(Eigen::Vector3d::Zero()), q(odom.q),
        yaw(uav_utils::get_yaw_from_quaternion(odom.q)), yaw_rate(0),
        head_rate(0){};
};
} // namespace mytest_contorller

#endif