#!/usr/bin/env python3.6

import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time


# pip3 install -r requirement.txt

LED_Pin = 19  # pin 19


def cmd_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " received message: %s", data.data)
    if data.data == 'Blink':
        # GPIO.output(LED_Pin, GPIO.LOW)
        # time.sleep(1)
        # GPIO.output(LED_Pin, GPIO.HIGH)
        count = 3
        rospy.loginfo('Blink 3 times.')
        while (count > 0):
            GPIO.output(LED_Pin, GPIO.LOW)
            time.sleep(1)
            GPIO.output(LED_Pin, GPIO.HIGH)
            time.sleep(1)
            count -= 1
        rospy.loginfo('ok')



def input_callback(event):
    str = input()
    if str == 'l' or str == 'L':
        count = 3
        rospy.loginfo('Blink 3 times.')
        while (count > 0):
            GPIO.output(LED_Pin, GPIO.LOW)
            time.sleep(1)
            GPIO.output(LED_Pin, GPIO.HIGH)
            time.sleep(1)
            count -= 1


if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('servo_controller_node', anonymous=True)
    rospy.Subscriber("/drop_cmd", String, cmd_callback)
    

    rospy.Timer(rospy.Duration(0.5), input_callback)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED_Pin, GPIO.OUT)
    GPIO.output(LED_Pin, GPIO.HIGH)
    mode = GPIO.getmode()


    print("GPIO Control Init.")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    GPIO.cleanup()
