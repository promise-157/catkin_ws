#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CircleTrajectory:
    def __init__(self):
        rospy.init_node('circle_trajectory_node', anonymous=True)
        
        # 你的 run_node.launch 里把 cmd 映射到了 planning/pos_cmd
        self.pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        
        # 用于在 RViz 中显示预期的圆形轨迹 (latch=True 表示后来的订阅者也能直接收到最后一条消息)
        self.path_pub = rospy.Publisher('/planning/expected_path', Path, queue_size=1, latch=True)
        
        # 实时记录并在 RViz 中显示无人机的实际飞行轨迹
        self.actual_path_pub = rospy.Publisher('/planning/actual_path', Path, queue_size=1)
        self.actual_path = Path()
        self.actual_path.header.frame_id = "map"
        
        self.start_x = None
        self.start_y = None
        self.start_z = None
        
        self.current_x = None
        self.current_y = None
        self.current_z = None
        
        # === 圆形轨迹参数 ===
        self.R = 2.0         # 圆的半径(米)
        self.T = 15.0        # 绕一圈所需的时间(秒) (15秒一圈)
        self.omega = 2.0 * math.pi / self.T
        # 动态捕捉起飞悬停高度，避免硬编码 1.0m 导致高度突变炸机
        
        # 订阅里程计以获取当前位置，避免起飞后猛烈切入圆心
        self.sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_cb)
        
        rospy.loginfo("Waiting for odom...")
        while not rospy.is_shutdown() and self.start_x is None:
            rospy.sleep(0.1)
            
        rospy.loginfo("Odom received. Ready to fly a circle!")
        
        # --- 仿照 kr_trackers_manager 引入状态机 ---
        self.state = "YAW_ALIGN" # 初始状态为对齐机头
        
        # 假设我们从当前的 X 位置前方 R 米处开始绕圈（当前点在圆周上）
        self.center_x = self.start_x - self.R
        self.center_y = self.start_y
        self.z_height = self.start_z   # 使用无人机的实际悬停高度
        
        # 发布用于 RViz 显示的预期轨迹
        self.publish_expected_path()
        
        self.start_time = rospy.Time.now().to_sec()
        
        # 按照 50Hz 频率发布控制指令
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_cb)

    def publish_expected_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        # 将一个完整周期分成 100 个点来绘制圆形路径
        num_points = 100
        for i in range(num_points + 1):
            t_sim = (self.T / float(num_points)) * i
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            
            pose.pose.position.x = self.center_x + self.R * math.cos(self.omega * t_sim)
            pose.pose.position.y = self.center_y + self.R * math.sin(self.omega * t_sim)
            pose.pose.position.z = self.z_height
            
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

    def odom_cb(self, msg):
        # 持续读取并更新实时位置
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        # 记录并发布无人机的实际飞行轨迹给 RViz
        self.actual_path.header.stamp = rospy.Time.now()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = msg.pose.pose
        self.actual_path.poses.append(pose_stamped)
        
        # 限制历史轨迹点数量为最多 3000 个（约60秒的轨迹长度），防止内存溢出导致卡顿
        if len(self.actual_path.poses) > 3000:
            self.actual_path.poses.pop(0)
            
        self.actual_path_pub.publish(self.actual_path)

        # 仅在第一次拿到信息时锁定为起始基准，如果不加这个限制条件，轨迹圆心会跟着实时位置乱飘
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_z = self.current_z
            
            # 【提取四元数获取初始偏航角】
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.start_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_cb(self, event):
        t_global = rospy.Time.now().to_sec()
        t = t_global - self.start_time
        
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"
        
        # ======================================================================
        # 以下逻辑仿照 kr_trackers_manager 和 kr_trackers 编写
        # 我们在这里手写一个轻量级的 Tracker 切换状态机 (Transition Machine)：
        # 1. YAW_ALIGN 状态：先在原地悬停，缓慢将偏航角对齐到圆弧切线的方向。
        # 2. TRACKING 状态 ：启动画圆，由于刚开始 v=0, a=0，所以前几秒采用平滑过渡系数缓慢加速。
        # ======================================================================
        
        if self.state == "YAW_ALIGN":
            align_duration = 3.0 # 原地转头耗时 3 秒
            
            if t >= align_duration:
                # 转头完成，进入画圆状态，并重置时间锚点
                self.state = "TRACKING"
                self.start_time = t_global
                t = 0.0
            else:
                # 【仿 kr_trackers 的 hover 状态 + line_tracker_yaw 的插值逻辑】
                # 在此期间，保持在起飞点绝对静止，不给出任何速度和加速度
                cmd.position.x = self.start_x
                cmd.position.y = self.start_y
                cmd.position.z = self.start_z
                cmd.velocity.x = 0.0; cmd.velocity.y = 0.0; cmd.velocity.z = 0.0
                cmd.acceleration.x = 0.0; cmd.acceleration.y = 0.0; cmd.acceleration.z = 0.0
                cmd.jerk.x = 0.0; cmd.jerk.y = 0.0; cmd.jerk.z = 0.0
                
                # 计算一条非线性的平滑插值曲线 (Min-Jerk 风格系数: 从0平滑过渡到1)
                ratio = t / align_duration
                smooth_ratio = 3 * (ratio**2) - 2 * (ratio**3) 
                
                target_yaw = math.pi / 2.0 # 圆起步瞬间的速度防线就是 90 度 (PI/2)
                # 计算两角的最短路径（防止无人机傻傻转一大圈）
                dyaw = math.atan2(math.sin(target_yaw - self.start_yaw), math.cos(target_yaw - self.start_yaw))
                
                cmd.yaw = self.start_yaw + dyaw * smooth_ratio
                # yaw_dot 代表角速度插值结果的导数
                cmd.yaw_dot = dyaw * (6 * ratio - 6 * ratio**2) / align_duration
                
                # 必须添加标志位，防止 px4ctrl 抛出 VEL_IN_BODY 或判定为无效轨迹
                cmd.trajectory_id = 1
                cmd.trajectory_flag = getattr(PositionCommand, 'TRAJECTORY_STATUS_READY', 1)
                
                self.pub.publish(cmd)
                return

        # ========= state == "TRACKING" : 正式开始画圆 =========
        # 【仿 kr_trackers 中 trajectory_tracker.cpp / traj_gen.cpp 动态调速平滑机制】
        accel_duration = 3.0 # 起步平滑加速耗时 3 秒
        
        if t < accel_duration:
            # 在前 3 秒内，不使用恒定的 self.omega，而是让实际角速度 nu 沿余弦曲线从 0 平滑爬坡到 self.omega
            nu = (self.omega / 2.0) * (1.0 - math.cos(math.pi * t / accel_duration))
            # alpha 是当前角加速度
            alpha = (self.omega * math.pi / (2.0 * accel_duration)) * math.sin(math.pi * t / accel_duration)
            # theta 是目前跑过的相位角 (积分得到)
            theta = (self.omega / 2.0) * t - (self.omega * accel_duration / (2.0 * math.pi)) * math.sin(math.pi * t / accel_duration)
        else:
            # 加速段结束，恢复匀速画圆
            nu = self.omega
            alpha = 0.0
            # 加上在前 3 秒加速段跑过的基础相位角
            theta_accel_end = (self.omega / 2.0) * accel_duration
            theta = theta_accel_end + self.omega * (t - accel_duration)
        
        # 位置 (Position)
        cmd.position.x = self.center_x + self.R * math.cos(theta)
        cmd.position.y = self.center_y + self.R * math.sin(theta)
        cmd.position.z = self.z_height
        
        # 速度 (Velocity) = 位置对时间的一阶导数 (由于速度不再恒定，这里用求出来的临时 nu 去算)
        cmd.velocity.x = -self.R * nu * math.sin(theta)
        cmd.velocity.y = self.R * nu * math.cos(theta)
        cmd.velocity.z = 0.0
        
        # 加速度 (Acceleration) = 链式求导，除了法向加速度，还有切向加速度 alpha
        cmd.acceleration.x = -self.R * (nu**2) * math.cos(theta) - self.R * alpha * math.sin(theta)
        cmd.acceleration.y = -self.R * (nu**2) * math.sin(theta) + self.R * alpha * math.cos(theta)
        cmd.acceleration.z = 0.0
        
        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = 0.0
        
        # 偏航角 (Yaw): 永远让机头朝向飞行轨迹的切线切向（机头朝前）
        cmd.yaw = theta + math.pi / 2.0
        cmd.yaw_dot = nu
        
        # 必须添加标志位，防止 px4ctrl 抛出 VEL_IN_BODY 或判定为无效轨迹
        cmd.trajectory_id = 1
        cmd.trajectory_flag = getattr(PositionCommand, 'TRAJECTORY_STATUS_READY', 1)
        
        self.pub.publish(cmd)

if __name__ == '__main__':
    try:
        CircleTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass