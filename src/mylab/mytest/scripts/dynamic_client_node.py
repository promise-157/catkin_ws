#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("参数已被更新为: {mode_bool}, {cmd_bool}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client_node")

    # 创建一个连接到你控制节点的 Client，第二个参数是发生改变时的回调函数
    client = dynamic_reconfigure.client.Client("/myctrl", timeout=30, config_callback=callback)
    
    # 模拟在运行时改变参数 (相当于拨动了 rqt_reconfigure 的拨杆)
    rospy.sleep(2.0)
    client.update_configuration({"mode_bool": True})
    
    rospy.sleep(2.0)
    client.update_configuration({"cmd_bool": True})
    
    rospy.spin()