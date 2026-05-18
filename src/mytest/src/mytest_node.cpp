#include <geometry_msgs/PoseStamped.h> //发布的消息体对应的头文件，该消息体的类型为geometry_msgs：：PoseStamped 本地位置
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h> //订阅的消息体的
#include <ros/ros.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "mytest_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::Rate rate(20.0);
  geometry_msgs::PoseStamped pos;
  pos.pose.position.x = 0;
  pos.pose.position.y = 0;
  pos.pose.position.z = 2;
  mavros_msgs::SetMode off_set_mode;
  off_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }
  //在进入OFFBOARD（机载/地面站控制）模式之前，必须已经接收到了控制期望值（Setpoint）
  for (int i = 100; ros::ok() && i > 0; i--) {
    local_pos_pub.publish(pos);
    ros::spinOnce();
    rate.sleep();
  }

  ros::Time last_request = ros::Time::now();
  while (ros::ok()) {
    if (current_state.mode != "OFFBOARD" &&
        ((ros::Time::now() - last_request) > ros::Duration(5.0))) {
      if (set_mode_client.call(off_set_mode) &&
          off_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if (!current_state.armed &&
          ((ros::Time::now() - last_request) > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    local_pos_pub.publish(pos);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}