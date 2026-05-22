#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 初始化cv_bridge
bridge = CvBridge()

def detect_aruco(frame):
    # 定义Aruco码字典为原始Aruco字典
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    # 检测Aruco码
    corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    # rospy.loginfo(" jin ru shi bie.")

    if ids is not None:
        # 绘制检测到的Aruco码边框和ID
        aruco.drawDetectedMarkers(frame, corners, ids)
        rospy.loginfo("Detected Aruco markers: %s", ids.flatten())
    else:
        rospy.loginfo("No Aruco markers detected.")

    # 显示检测结果
    cv2.imshow("Aruco Detection (Original)", frame)
    cv2.waitKey(1)

def image_callback(msg):
    try:
        # 使用cv_bridge将ROS的Image消息转为OpenCV格式
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 进行Aruco码检测
        detect_aruco(frame)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", str(e))

def main():
    rospy.init_node('original_aruco_detector_node', anonymous=True)

    # 订阅摄像头图像话题
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    rospy.loginfo("Original Aruco marker detector started.")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Aruco detector.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
