#include "mytest/PX4Ctrlfsm.h"
#include "mytest/PX4Ctrlparam.h"
#include "mytest/controller.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>
#include <signal.h>
using namespace mytest_controller;
using namespace mytest_px4ctrlfsm;
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "myctrl");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~"); //使用私有命名空间

  mytest_px4ctrlparam::Parameter_t param;
  param.config_from_ros_handle(nh_);

  // Controller controller(param);
  LinearControl controller(param);
  PX4CtrlFSM fsm(param, controller);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
      "mavros/state", 10,
      boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

  ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>(
      "mavros/extended_state", 10,
      boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "odom", 100, boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
      "cmd", 100, boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
      "mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
      100, boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  ros::Subscriber rc_sub;
  if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages
                                 // although no RC is connected
  {
    rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "mavros/rc/in", 10, boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
  }

  /*判断参数选择是否订阅遥控器*/

  // ros::Subscriber bat_sub =
  //     nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
  //                                             100,
  //                                             boost::bind(&Battery_Data_t::feed,
  //                                             &fsm.bat_data, _1),
  //                                             ros::VoidConstPtr(),
  //                                             ros::TransportHints().tcpNoDelay());

  ros::Subscriber takeoff_land_sub = nh.subscribe<quadrotor_msgs::TakeoffLand>(
      "takeoff_land", 100,
      boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  /*订阅起落模式,状态机模式随意初始状态就在不停的pub了所以可以解锁到offboard*/

  fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "mavros/setpoint_raw/attitude", 10);
  fsm.traj_start_trigger_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

  fsm.debug_pub =
      nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

  fsm.set_FCU_mode_srv =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  fsm.arming_client_srv =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  fsm.reboot_FCU_srv =
      nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  /*给状态机中的成员类对象绑定成话题发布者，
  飞控单元发布绑定为maros中目标海拔msgs；
  开始轨迹发布绑定几何功能包中的posestamped的msgs
  debug
  飞控单元：，设置模式，是否上锁arming，是否重启的服务

  */

  ros::Duration(0.5).sleep();
  /*阻塞0.5秒*/
  dynamic_reconfigure::Server<mytest::fake_rcConfig> server;
  dynamic_reconfigure::Server<mytest::fake_rcConfig>::CallbackType f;

  /*动态重配置
  dynamic_reconfigure是命名空间，到时候有动态配置需求再来研究

  */

  if (param.takeoff_land.no_RC) {
    f = boost::bind(&Dynamic_Data_t::feed, &fsm.dy_data, _1); //绑定回调函数
    server.setCallback(
        f); //为服务器设置回调函数，
            //节点程序运行时会调用一次回调函数来输出当前的参数配置情况
    ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
  } else {
    ROS_INFO("PX4CTRL] Waiting for RC");
    while (ros::ok()) {
      ros::spinOnce();
      if (fsm.rc_is_received(ros::Time::now())) {
        ROS_INFO("[PX4CTRL] RC received.");
        break;
      }
      ros::Duration(0.1).sleep();
    }
  }

  /*运行方式判断*/

  int trials = 0;
  while (ros::ok() && !fsm.state_data.current_state.connected) {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    if (trials++ > 5)
      ROS_ERROR("Unable to connnect to PX4!!!");
  }

  ros::Rate r(param.ctrl_freq_max);
  while (ros::ok()) {
    ROS_INFO_ONCE("PX4CTRL] Is OK!");
    r.sleep();
    ros::spinOnce();
    fsm.process(); // We DO NOT rely on feedback as trigger, since there is no
                   // significant performance difference through our test.
  }
  /*总结：基本都不需要改动，有就是fsm，回调函数需要改动*/
}