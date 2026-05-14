import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv2 import aruco

class RealSenseArucoDetector:
    def __init__(self):
        rospy.init_node('realsense_aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # 使用 DICT_ARUCO_ORIGINAL 字典
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        # 设置检测参数
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params.minMarkerPerimeterRate = 0.05
        self.aruco_params.minCornerDistanceRate = 0.1
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.polygonalApproxAccuracyRate = 0.03

        # 订阅相机话题
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        try:
            # 转换ROS图像消息为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 检测 ArUco 码
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            valid_detection = False  # 标记是否检测到有效的 ArUco 码
            for i, corner in enumerate(corners):
                # 计算四边形的面积
                area = cv2.contourArea(corner)
                if area >= 1000:  # 面积大于设定阈值时，视为有效
                    valid_detection = True
                    center_x = int((corner[0][0][0] + corner[0][2][0]) / 2)
                    center_y = int((corner[0][0][1] + corner[0][2][1]) / 2)
                    aruco_id = ids[i][0]
                    rospy.loginfo(f"Detected ArUco ID: {aruco_id} at position ({center_x}, {center_y}), area: {area}")
                    break  # 检测到有效的 ArUco 码后停止
            if not valid_detection:
                rospy.loginfo("No valid ArUco markers detected.")
        else:
            rospy.loginfo("No ArUco markers detected.")

        # 显示图像窗口
        cv2.imshow("RealSense ArUco Detection", frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = RealSenseArucoDetector()
    detector.run()
