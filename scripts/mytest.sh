#!/bin/bash

# ==============================================================================
# 1. 信号捕获函数：当收到 Ctrl+C (SIGINT) 或 脚本退出 (SIGTERM) 时触发
# ==============================================================================
cleanup() {
    echo -e "\n[INFO] Detecting Ctrl+C, cleaning up all background processes..."
    
    # 杀掉当前脚本进程组下的所有子进程 (推荐做法)
    # pkill -P $$ 能够确保把当前脚本启动的所有后台 roslaunch/rosrun 一网打尽
    pkill -P $$
    
    # 或者保险起见，直接干掉 ros 的核心大户（选配，按需放开）
    # killall -9 gzserver gzclient rosmaster 2>/dev/null
    
    exit 0
}

# 绑定信号：一旦脚本运行中遭遇 Ctrl+C，立刻执行 cleanup 函数
trap cleanup SIGINT SIGTERM

# ==============================================================================
# 2. 正常启动流程
# ==============================================================================

# 启动仿真环境（后台运行，记录 PID 只是为了以防万一）
echo "Launching Gazebo and Mavros..."
roslaunch mytest run_mavros_gazebo.launch & 
PID_GAZEBO=$! # 获取上一步后台进程的 PID

# 等待仿真完全加载
sleep 40;

# 1. 触发自动起飞
echo "Triggering auto takeoff..."
rostopic pub -1 /takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1" & 
sleep 40;

# 2. 修改动态参数进入 hover_mode 和 command_mode 
echo "Switching to AUTO_HOVER and CMD_CTRL mode..."
rosrun dynamic_reconfigure dynparam set /myctrl mode_bool True & sleep 4;
rosrun dynamic_reconfigure dynparam set /myctrl cmd_bool True & sleep 4;

# ==============================================================================
# 3. 阻塞前台，防止脚本直接结束退出
# ==============================================================================
echo "All nodes started successfully. Press [Ctrl+C] to stop all nodes."

# 这里使用 wait 阻塞前台。此时按下 Ctrl+C 会被上面的 trap 捕获，从而完美触发 cleanup
wait