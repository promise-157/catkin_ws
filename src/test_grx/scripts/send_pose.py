#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# 订阅 /Odometry 话题并将数据转发给飞控
def odom_callback(data):
    pose_msg = PoseStamped()
    
    # 填充位置信息
    pose_msg.header = data.header
    pose_msg.pose.position = data.pose.pose.position
    
    # 填充姿态信息
    pose_msg.pose.orientation = data.pose.pose.orientation

    # 发布位姿到 /mavros/vision_pose/pose 话题
    pose_pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_to_mavros')
    
    # 发布到飞控的位姿话题
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    
    # 订阅 LiDAR 生成的 /Odometry 话题
    rospy.Subscriber('/Odometry', Odometry, odom_callback)
    
    rospy.spin()
