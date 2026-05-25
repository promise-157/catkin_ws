// #include <iostream>
// #include <myplan_manage/planner_manager.h>
// #include <ros/ros.h>
// using namespace std;
// namespace myego {
// // SECTION interfaces for setup and query

// EGOPlannerManager::EGOPlannerManager() {}

// EGOPlannerManager::~EGOPlannerManager() {}

// // !SECTION

// // SECTION rebond replanning

// bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt,
//                                       Eigen::Vector3d start_vel,
//                                       Eigen::Vector3d start_acc,
//                                       Eigen::Vector3d local_target_pt,
//                                       Eigen::Vector3d local_target_vel,
//                                       bool flag_polyInit,
//                                       bool flag_randomPolyTraj) {
//   static int count = 0;
//   printf("\033[47;30m\n[drone %d replan "
//          "%d]==============================================\033[0m\n",
//          pp_.drone_id, count++);
//   // cout.precision(3);
//   // cout << "start: " << start_pt.transpose() << ", " <<
//   start_vel.transpose()
//   // << "\ngoal:" << local_target_pt.transpose() << ", " <<
//   // local_target_vel.transpose()
//   //      << endl;

//   if ((start_pt - local_target_pt).norm() < 0.2) {
//     cout << "Close to goal" << endl;
//     continous_failures_count_++;
//     return false;
//   }

//   bspline_optimizer_->setLocalTargetPt(local_target_pt);

//   ros::Time t_start = ros::Time::now();
//   ros::Duration t_init, t_opt, t_refine;

//   /*** STEP 1: INIT ***/
//   double ts = (start_pt - local_target_pt).norm() > 0.1
//                   ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5
//                   : pp_.ctrl_pt_dist / pp_.max_vel_ *
//                         5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense,
//                         and
//                            // will surely exceed the acc/vel limits
//   vector<Eigen::Vector3d> point_set, start_end_derivatives;
//   static bool flag_first_call = true, flag_force_polynomial = false;
//   bool flag_regenerate = false;
//   do {
//     point_set.clear();
//     start_end_derivatives.clear();
//     flag_regenerate = false;

//     if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| (
//     start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated
//     from a min-snap traj by order.
//       {
//       flag_first_call = false;
//       flag_force_polynomial = false;

//       PolynomialTraj gl_traj;

//       double dist = (start_pt - local_target_pt).norm();
//       double time =
//           pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist
//               ? sqrt(dist / pp_.max_acc_)
//               : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ +
//                     2 * pp_.max_vel_ / pp_.max_acc_;

//       if (!flag_randomPolyTraj) {
//         gl_traj = PolynomialTraj::one_segment_traj_gen(
//             start_pt, start_vel, start_acc, local_target_pt,
//             local_target_vel, Eigen::Vector3d::Zero(), time);
//       } else {
//         Eigen::Vector3d horizen_dir =
//             ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1)))
//                 .normalized();
//         Eigen::Vector3d vertical_dir =
//             ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
//         Eigen::Vector3d random_inserted_pt =
//             (start_pt + local_target_pt) / 2 +
//             (((double)rand()) / RAND_MAX - 0.5) *
//                 (start_pt - local_target_pt).norm() * horizen_dir * 0.8 *
//                 (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
//             (((double)rand()) / RAND_MAX - 0.5) *
//                 (start_pt - local_target_pt).norm() * vertical_dir * 0.4 *
//                 (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
//         Eigen::MatrixXd pos(3, 3);
//         pos.col(0) = start_pt;
//         pos.col(1) = random_inserted_pt;
//         pos.col(2) = local_target_pt;
//         Eigen::VectorXd t(2);
//         t(0) = t(1) = time / 2;
//         gl_traj =
//             PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel,
//                                         start_acc, Eigen::Vector3d::Zero(),
//                                         t);
//       }

//       double t;
//       bool flag_too_far;
//       ts *= 1.5; // ts will be divided by 1.5 in the next
//       do {
//         ts /= 1.5;
//         point_set.clear();
//         flag_too_far = false;
//         Eigen::Vector3d last_pt = gl_traj.evaluate(0);
//         for (t = 0; t < time; t += ts) {
//           Eigen::Vector3d pt = gl_traj.evaluate(t);
//           if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5) {
//             flag_too_far = true;
//             break;
//           }
//           last_pt = pt;
//           point_set.push_back(pt);
//         }
//       } while (flag_too_far ||
//                point_set.size() <
//                    7); // To make sure the initial path has enough points.
//       t -= ts;
//       start_end_derivatives.push_back(gl_traj.evaluateVel(0));
//       start_end_derivatives.push_back(local_target_vel);
//       start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
//       start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
//     } else // Initial path generated from previous trajectory.
//     {

//       double t;
//       double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

//       vector<double> pseudo_arc_length;
//       vector<Eigen::Vector3d> segment_point;
//       pseudo_arc_length.push_back(0.0);
//       for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts) {
//         segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
//         if (t > t_cur) {
//           pseudo_arc_length.push_back(
//               (segment_point.back() - segment_point[segment_point.size() -
//               2])
//                   .norm() +
//               pseudo_arc_length.back());
//         }
//       }
//       t -= ts;

//       double poly_time =
//           (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt)
//               .norm() /
//           pp_.max_vel_ * 2;
//       if (poly_time > ts) {
//         PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(
//             local_data_.position_traj_.evaluateDeBoorT(t),
//             local_data_.velocity_traj_.evaluateDeBoorT(t),
//             local_data_.acceleration_traj_.evaluateDeBoorT(t),
//             local_target_pt, local_target_vel, Eigen::Vector3d::Zero(),
//             poly_time);

//         for (t = ts; t < poly_time; t += ts) {
//           if (!pseudo_arc_length.empty()) {
//             segment_point.push_back(gl_traj.evaluate(t));
//             pseudo_arc_length.push_back(
//                 (segment_point.back() - segment_point[segment_point.size() -
//                 2])
//                     .norm() +
//                 pseudo_arc_length.back());
//           } else {
//             ROS_ERROR("pseudo_arc_length is empty, return!");
//             continous_failures_count_++;
//             return false;
//           }
//         }
//       }

//       double sample_length = 0;
//       double cps_dist =
//           pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the
//           next
//       size_t id = 0;
//       do {
//         cps_dist /= 1.5;
//         point_set.clear();
//         sample_length = 0;
//         id = 0;
//         while ((id <= pseudo_arc_length.size() - 2) &&
//                sample_length <= pseudo_arc_length.back()) {
//           if (sample_length >= pseudo_arc_length[id] &&
//               sample_length < pseudo_arc_length[id + 1]) {
//             point_set.push_back(
//                 (sample_length - pseudo_arc_length[id]) /
//                     (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) *
//                     segment_point[id + 1] +
//                 (pseudo_arc_length[id + 1] - sample_length) /
//                     (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) *
//                     segment_point[id]);
//             sample_length += cps_dist;
//           } else
//             id++;
//         }
//         point_set.push_back(local_target_pt);
//       } while (
//           point_set.size() <
//           7); // If the start point is very close to end point, this will
//           help

//       start_end_derivatives.push_back(
//           local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
//       start_end_derivatives.push_back(local_target_vel);
//       start_end_derivatives.push_back(
//           local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
//       start_end_derivatives.push_back(Eigen::Vector3d::Zero());

//       if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist *
//                                  3) // The initial path is unnormally too
//                                  long!
//       {
//         flag_force_polynomial = true;
//         flag_regenerate = true;
//       }
//     }
//   } while (flag_regenerate);

//   Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
//   UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives,
//                                         ctrl_pts);

//   vector<std::pair<int, int>> segments;
//   segments =
//       bspline_optimizer_->initControlPoints(ctrl_pts, true); // 通过A star
//       搜索

//   t_init = ros::Time::now() - t_start;
//   t_start = ros::Time::now();

//   /*** STEP 2: OPTIMIZE ***/
//   bool flag_step_1_success = false;
//   vector<vector<Eigen::Vector3d>> vis_trajs;

//   if (pp_.use_distinctive_trajs) {
//     // cout << "enter" << endl;
//     std::vector<ControlPoints> trajs =
//         bspline_optimizer_->distinctiveTrajs(segments);
//     cout << "\033[1;33m"
//          << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

//     double final_cost, min_cost = 999999.0;
//     for (int i = trajs.size() - 1; i >= 0; i--) {
//       if (bspline_optimizer_->BsplineOptimizeTrajRebound(
//               ctrl_pts_temp, final_cost, trajs[i], ts)) {

//         cout << "traj " << trajs.size() - i << " success." << endl;

//         flag_step_1_success = true;
//         if (final_cost < min_cost) {
//           min_cost = final_cost;
//           ctrl_pts = ctrl_pts_temp;
//         }

//         // visualization
//         point_set.clear();
//         for (int j = 0; j < ctrl_pts_temp.cols(); j++) {
//           point_set.push_back(ctrl_pts_temp.col(j));
//         }
//         vis_trajs.push_back(point_set);
//       } else {
//         cout << "traj " << trajs.size() - i << " failed." << endl;
//       }
//     }

//     t_opt = ros::Time::now() - t_start;

//     visualization_->displayMultiInitPathList(
//         vis_trajs,
//         0.2); // This visuallization will take up several milliseconds.
//   } else {
//     flag_step_1_success =
//         bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
//     t_opt = ros::Time::now() - t_start;
//     // static int vis_id = 0;
//     visualization_->displayInitPathList(point_set, 0.2, 0);
//   }

//   cout << "plan_success=" << flag_step_1_success << endl;
//   if (!flag_step_1_success) {
//     visualization_->displayOptimalList(ctrl_pts, 0);
//     continous_failures_count_++;
//     return false;
//   }

//   t_start = ros::Time::now();

//   UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
//   pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
//   pp_.feasibility_tolerance_);

//   /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
//   // Note: Only adjust time in single drone mode. But we still allow drone_0
//   to
//   // adjust its time profile.
//   if (pp_.drone_id <= 0) {

//     double ratio;
//     bool flag_step_2_success = true;
//     if (!pos.checkFeasibility(ratio, false)) {
//       cout << "Need to reallocate time." << endl;

//       Eigen::MatrixXd optimal_control_points;
//       flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio,
//                                            ts, optimal_control_points);
//       if (flag_step_2_success)
//         pos = UniformBspline(optimal_control_points, 3, ts);
//     }

//     if (!flag_step_2_success) {
//       printf("\033[34mThis refined trajectory hits obstacles. It doesn't "
//              "matter if appeares occasionally. But if continously appearing,
//              " "Increase parameter \"lambda_fitness\".\n\033[0m");
//       continous_failures_count_++;
//       return false;
//     }
//   } else {
//     static bool print_once = true;
//     if (print_once) {
//       print_once = false;
//       ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
//     }
//   }

//   t_refine = ros::Time::now() - t_start;

//   // save planned results
//   updateTrajInfo(pos, ros::Time::now());

//   static double sum_time = 0;
//   static int count_success = 0;
//   sum_time += (t_init + t_opt + t_refine).toSec();
//   count_success++;
//   cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec()
//        << "\033[0m,optimize:" << (t_init + t_opt).toSec()
//        << ",refine:" << t_refine.toSec()
//        << ",avg_time=" << sum_time / count_success << endl;

//   // success. YoY
//   continous_failures_count_ = 0;
//   return true;
// }

// } // namespace myego