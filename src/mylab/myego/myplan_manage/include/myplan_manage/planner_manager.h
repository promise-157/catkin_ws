#ifndef MYPLANNER_MANAGER_H_
#define MYPLANNER_MANAGER_H_
#include <Eigen/Dense>
#include <memory>
#include <mytraj_utils/plan_container.hpp>
using namespace std;

namespace myego {
class EGOPlannerManager {
  // SECTION stable
public:
  EGOPlannerManager();
  ~EGOPlannerManager();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* main planning interface */
  bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                     Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                     Eigen::Vector3d end_vel, bool flag_polyInit,
                     bool flag_randomPolyTraj);
  PlanParameters pp_;
};
} // namespace myego
#endif