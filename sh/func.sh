#!/bin/bash


gnome-terminal -x bash -c "cd /home/nvidia/catkin_ws/src/Challege_ROS/object_det/scripts; python3 det.py; exec bash;"
sleep 5s
gnome-terminal -x bash -c  "cd /home/nvidia/catkin_ws/src/Challege_ROS/recognize_aruco; python3 image.py; exec bash;" 
sleep 3s
gnome-terminal -x bash -c "roslaunch faster_lio rflysim.launch; exec bash;"
sleep 3s
gnome-terminal -x bash -c  "roslaunch ego_planner rflysim.launch; exec bash;"
# sleep 5s
#gnome-terminal -x bash -c  "roslaunch controller controll.launch; exec bash;"
