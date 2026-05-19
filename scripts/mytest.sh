#roslaunch px4 fast_racing.launch & sleep 20;
# roslaunch ego_planner single_run_in_gazebo.launch & sleep 10;
roslaunch mytest run_mavros_gazebo.launch & sleep 30;

# 1. 触发自动起飞 (内部实现了自动解锁并起飞到预设高度进入 AUTO_HOVER)
echo "Triggering auto takeoff..."
rostopic pub -1 /takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1" & sleep 18;

# 2. 修改动态参数进入 hover_mode 和 command_mode 
# (注意这里节点名取决于 run_node.launch 中的实际名称，这里假设叫 myctrl)
echo "Switching to AUTO_HOVER and CMD_CTRL mode..."
rosrun dynamic_reconfigure dynparam set /myctrl mode_bool True & sleep 2;
rosrun dynamic_reconfigure dynparam set /myctrl cmd_bool True & sleep 2;

# roslaunch ego_planner rviz.launch
