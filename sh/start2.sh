#!/bin/bash

roscore &
sleep 5

cd /home/promise/catkin_ws/src/Challege_ROS/object_det/scripts && python3 det.py &
cd /home/promise/catkin_ws/src/Challege_ROS/recognize_aruco && python3 image.py & 


gnome-terminal --tab --title="usb" -e 'bash -c "echo nvidia | sudo -S uhubctl  -a cycle -d 5 -p 1-4; exec bash"'
sleep 10
#cd ../sensor_pkg && python3 main.py 

gnome-terminal --tab --title="mavros" -e 'bash -c "roslaunch mavros px4.launch; exec bash"'  
sleep 5
gnome-terminal --tab --title="drone state" -e 'bash -c "rostopic echo /mavros/state; exec bash"'  
gnome-terminal --tab --title="drone pose" -e 'bash -c "rostopic echo /mavros/local_position/pose; exec bash"'  
gnome-terminal --tab --title="target pose" -e 'bash -c "rostopic echo /mavros/setpoint_raw/local; exec bash"'  
gnome-terminal --tab --title="lio_output" -e 'bash -c "rostopic echo /mavros/odometry/out; exec bash"'
gnome-terminal --window --title="usb_cam" -e 'bash -c "roslaunch usb_cam usb_cam-test.launch; exec bash;"'
sleep 3
gnome-terminal --window --title="rs_camera" -e 'bash -c "roslaunch realsense2_camera rs_camera.launch; exec bash;"'
sleep 3
gnome-terminal --window --title="livox-mid360" -e 'bash -c "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash;"'
sleep 5
gnome-terminal --window --title="faster-lio" -e 'bash -c "roslaunch faster_lio rflysim.launch; exec bash;"'
sleep 5
gnome-terminal --window --title="rviz" -e 'bash -c "rosrun rviz rviz -d /home/nvidia/rflysim.rviz;"'
sleep 5
gnome-terminal --window --title="planner" -e 'bash -c "roslaunch ego_planner rflysim.launch; exec bash;"'
sleep 10
gnome-terminal --window --title="control" -e 'bash -c "roslaunch controller real_controll.launch; exec bash;"' 


