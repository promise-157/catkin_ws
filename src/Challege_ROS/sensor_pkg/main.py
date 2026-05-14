
# import required libraries
# pip3 install pymavlink pyserial

import cv2
import numpy as np
import time
import VisionCaptureApi
import math


# 启用ROS发布模式
VisionCaptureApi.isEnableRosTrans = True
vis = VisionCaptureApi.VisionCaptureApi()

# VisionCaptureApi 中的配置函数
vis.jsonLoad(jsonPath = "/home/nvidia/catkin_ws/src/Challege_ROS/sensor_pkg/Config.json")  # 加载Config.json中的传感器配置文件
vis.startImgCap()  # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
print('Start Image Reciver')

vis.RemotSendIP = "192.168.2.5"

vis.sendImuReqCopterSim(IP="192.168.2.3")
