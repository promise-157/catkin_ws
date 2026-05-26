#ifndef MYPLANNER_MANAGER_H_
#define MYPLANNER_MANAGER_H_
#include <Eigen/Dense>
#include <memory>
#include <mybspline_opt/bspline_optimizer.h>
#include <mytraj_utils/plan_container.hpp>
#include <mytraj_utils/planning_visualization.h>
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

  void initPlanModules(ros::NodeHandle &nh,
                       PlanningVisualization::Ptr vis = NULL);
  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  GridMap::Ptr grid_map_;
  myego::ObjPredictor::Ptr obj_predictor_;
  SwarmTrajData swarm_trajs_buf_;

  bool EmergencyStop(Eigen::Vector3d stop_pos);
  bool planGlobalTraj(const Eigen::Vector3d &start_pos,
                      const Eigen::Vector3d &start_vel,
                      const Eigen::Vector3d &start_acc,
                      const Eigen::Vector3d &end_pos,
                      const Eigen::Vector3d &end_vel,
                      const Eigen::Vector3d &end_acc);
  bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &start_acc,
                               const std::vector<Eigen::Vector3d> &waypoints,
                               const Eigen::Vector3d &end_vel,
                               const Eigen::Vector3d &end_acc);

  void deliverTrajToOptimizer(void) {
    bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_);
  };

  void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

  double getSwarmClearance(void) {
    return bspline_optimizer_->getSwarmClearance();
  }

  bool checkCollision(int drone_id);

private:
  /* main planning algorithms & modules */
  PlanningVisualization::Ptr visualization_;
  // ros::Publisher obj_pub_; //zx-todo

  BsplineOptimizer::Ptr bspline_optimizer_;
  int continous_failures_count_{0};

  bool refineTrajAlgo(UniformBspline &traj,
                      vector<Eigen::Vector3d> &start_end_derivative,
                      double ratio, double &ts,
                      Eigen::MatrixXd &optimal_control_points);
  void reparamBspline(UniformBspline &bspline,
                      vector<Eigen::Vector3d> &start_end_derivative,
                      double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                      double &time_inc);

  void updateTrajInfo(const UniformBspline &position_traj,
                      const ros::Time time_now);

public:
  typedef unique_ptr<EGOPlannerManager> Ptr;
};
} // namespace myego
#endif