这里将会写一下px4是如何控制无人机的。
# 控制消息
px4的串级pid为：位置环--速度环--姿态角度环（外环，输出期望角速度）姿态角速度环（内环，输出扭矩）--扭矩或推力
## mavros_msgs::AttitudeTarget,掩码采用四元数则px4负责姿态角度环和角速度环，掩码采用角速度则px4负责角速度环
话题：mavros/setpoint_raw/attitude
1. type_mask：掩码选择控制角度还是角速度
2. orientation：目标四元数
3. thrust：总合力,推力，0-1
4. body_rate：翻滚速率（Roll rate）、俯仰速率（Pitch rate）和偏航速率（Yaw rate），单位是 rad/s
## mavros_msgs/PositionTarget，px4负责速度环，速度环，角度环，姿态环
话题：mavros/setpoint_raw/local
1. type_mask：自由组合成“纯速度控制”、“位置+速度控制”或“加速度控制”。
2. coordinate_frame：8为机体坐标系
3. position：目标位置
4. velocity：目标速度
5. acceleration_or_force：目标加速度
6. yaw：目标航向角
7. yaw_rate：目标航向角速度前馈
## geometry_msgs/PoseStamped，px4负责位置，速度，角度环，，姿态环
话题：mavros/setpoint_position/local
1. position：目标位置
2. orientation：目标姿态
## mavros_msgs::ActuatorControl，直接控制电机
话题：mavros/actuator_control
## mavros_msgs::GlobalPositionTarget，输入经纬度和海拔
话题：mavros/setpoint_raw/global
## 控制流程例子
【Ego-planner（规划层）】 ───[期望位置/速度/加速度]───► 【px4ctrl (LQR控制层)】 ───[目标四元数+推力]───► 【PX4飞控】

