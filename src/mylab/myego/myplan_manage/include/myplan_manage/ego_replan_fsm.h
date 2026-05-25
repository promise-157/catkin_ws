#ifndef MYEBO_REPLAN_FSM_H_
#define MYEBO_REPLAN_FSM_H_
#include "myplan_manage/planner_manager.h"
#include <ros/ros.h>

namespace myego {
class EGOReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    SEQUENTIAL_START
  };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  // EGOPlannerManager::Ptr planner_manager_;
};
} // namespace myego

#endif