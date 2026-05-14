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
  return 0;
}