#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

class CircleTrajectory:
    def __init__(self):
        rospy.init_node('circle_trajectory_node', anonymous=True)
        
        # 你的 run_node.launch 里把 cmd 映射到了 planning/pos_cmd
        self.pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        
        self.start_x = None
        self.start_y = None
        self.start_z = None
        
        # === 圆形轨迹参数 ===
        self.R = 2.0         # 圆的半径(米)
        self.T = 15.0        # 绕一圈所需的时间(秒) (15秒一圈)
        self.omega = 2.0 * math.pi / self.T
        self.z_height = 1.0  # 巡航高度
        
        # 订阅里程计以获取当前位置，避免起飞后猛烈切入圆心
        self.sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_cb)
        
        rospy.loginfo("Waiting for odom...")
        while not rospy.is_shutdown() and self.start_x is None:
            rospy.sleep(0.1)
            
        rospy.loginfo("Odom received. Ready to fly a circle!")
        
        # 假设我们从当前的 X 位置前方 R 米处开始绕圈（当前点在圆周上）
        self.center_x = self.start_x - self.R
        self.center_y = self.start_y
        
        self.start_time = rospy.Time.now().to_sec()
        
        # 按照 50Hz 频率发布控制指令
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_cb)

    def odom_cb(self, msg):
        if self.start_x is None:
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.start_z = msg.pose.pose.position.z

    def timer_cb(self, event):
        t = rospy.Time.now().to_sec() - self.start_time
        
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        
        # 位置 (Position)
        cmd.position.x = self.center_x + self.R * math.cos(self.omega * t)
        cmd.position.y = self.center_y + self.R * math.sin(self.omega * t)
        cmd.position.z = self.z_height
        
        # 速度 (Velocity) = 位置对时间的一阶导数
        cmd.velocity.x = -self.R * self.omega * math.sin(self.omega * t)
        cmd.velocity.y = self.R * self.omega * math.cos(self.omega * t)
        cmd.velocity.z = 0.0
        
        # 加速度 (Acceleration) = 速度对时间的一阶导数 (非常重要， px4ctrl极其依赖前馈加速度)
        cmd.acceleration.x = -self.R * (self.omega**2) * math.cos(self.omega * t)
        cmd.acceleration.y = -self.R * (self.omega**2) * math.sin(self.omega * t)
        cmd.acceleration.z = 0.0
        
        # 加加速度 (Jerk) 
        cmd.jerk.x = self.R * (self.omega**3) * math.sin(self.omega * t)
        cmd.jerk.y = -self.R * (self.omega**3) * math.cos(self.omega * t)
        cmd.jerk.z = 0.0
        
        # 偏航角 (Yaw): 永远让机头朝向飞行轨迹的切线切向（机头朝前）
        cmd.yaw = math.atan2(cmd.velocity.y, cmd.velocity.x)
        cmd.yaw_dot = self.omega
        
        self.pub.publish(cmd)

if __name__ == '__main__':
    try:
        CircleTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass