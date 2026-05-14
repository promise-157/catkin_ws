#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from common_msgs.msg import Aruco
from common_msgs.msg import MissionState
import rospy
import numpy as np



task = 0
# 获取 ArUco 字典
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

# 创建 ArUco 检测器
parameters = aruco.DetectorParameters_create()
nums = 1

def task_cb(msg):
    global task
    task = msg.task


msg_pub = None
def ImgCB(msg):
    global parameters
    global aruco_dict
    global task
    print("state",task)
    if(task < 9): #还没到二维码检测的地方不做处理
        return
    
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    # 检测标记
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    # 如果检测到了标记
       
    if ids is not None and len(ids) < 2:
        # 绘制检测到的标记
        # if len(ids) > 1:
        #     print("detect multi aruco")
        #     return
        image = aruco.drawDetectedMarkers(image, corners, ids)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_color = (0, 0,255)
        font_thickness = 2
        cv2.putText(image, str(ids[0]), tuple([50,50]), font, font_scale, font_color, font_thickness)
        root_path = "/home/nvidia/catkin_ws/PostRerurn_img/"
        img_name = "downView.jpg"
        cv2.imwrite(root_path+img_name,image)

        aruco_msg = Aruco()
        aruco_msg.header = msg.header
        aruco_msg.id = int(ids[0])
        aruco_msg.cnt_x = int(corners[0][0][0][0])
        aruco_msg.cnt_y = int(corners[0][0][0][1])
        msg_pub.publish(aruco_msg)
        
        # 显示图像
        # cv2.destroyAllWindows()
    else:
        print("No down ARuco markers detected.")
    cv2.imshow('down', image)
    cv2.waitKey(1)

def Img_preCB(msg):
    global parameters
    global aruco_dict
    global task
    global nums
    print("state",task)
    if(task<3 or task >8): #还没到二维码检测的地方不做处理
        return
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    if ids is not None and len(ids) < 2:
        image = aruco.drawDetectedMarkers(image, corners, ids)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_color = (0, 0,255)
        font_thickness = 2
        cv2.putText(image, str(ids[0]), tuple([50,50]), font, font_scale, font_color, font_thickness)

        root_path = "/home/nvidia/catkin_ws/PreReturn_img/"
        # str(task)+img_name
        img_name = "_PreView"

        suffix = '.jpg'
        if(task>=3 and task <=8):
            cv2.imwrite(root_path+str(int(task)-2)+img_name+str(nums)+suffix,image)
        nums+=1
        if(nums>3):
            nums=1
    else:
        print("No pre ARuco markers detected.")
    cv2.imshow('Pre', image)
    cv2.waitKey(1)
        
    
if __name__ == "__main__":
    rospy.init_node("aruco")
    msg_pub = rospy.Publisher("/Aruco", Aruco, queue_size = 10)
    
    # img_sub = rospy.Subscriber("/usb_cam/image_raw",Image,ImgCB)
    imgPre_sub = rospy.Subscriber("/camera/color/image_raw",Image,Img_preCB)
    task_sub = rospy.Subscriber("/task_state",MissionState,task_cb)

    rospy.spin()
