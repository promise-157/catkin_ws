#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <myplan_manage/ego_replan_fsm.h>

using namespace myego;

int main(int argc, char **argv) {

  ros::init(argc, argv, "myego_node");
  ros::NodeHandle nh("~");

  // EGOReplanFSM rebo_replan;

  // rebo_replan.init(nh);

  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}