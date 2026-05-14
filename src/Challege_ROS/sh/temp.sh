#!/bin/bash

roscore &
sleep 5
cd ../object_det/scripts && python3 det.py &
cd /home/nvidia/catkin_ws/src/Challege_ROS/recognize_aruco && python3 image.py & 
cd /home/nvidia/catkin_ws/src/Challege_ROS/sensor_pkg && python3 main.py &
sleep 5


#cd ../sensor_pkg && python3 main.py 
roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.2.3:20100" &

sleep 5
gnome-terminal --tab --title="drone state" -e 'bash -c "rostopic echo /mavros/state; exec bash"'  
gnome-terminal --tab --title="drone pose" -e 'bash -c "rostopic echo /mavros/local_position/pose; exec bash"'  
gnome-terminal --tab --title="target pose" -e 'bash -c "rostopic echo /mavros/setpoint_raw/local; exec bash"'  
gnome-terminal --tab --title="lio_output" -e 'bash -c "rostopic echo /mavros/odometry/out; exec bash"'  
sleep 10
gnome-terminal --window --title="faster-lio" -e 'bash -c "roslaunch faster_lio rflysim.launch; exec bash;"'
sleep 10
gnome-terminal --window --title="planner" -e 'bash -c "roslaunch ego_planner rflysim.launch; exec bash;"'
sleep 15
gnome-terminal --window --title="control" -e 'bash -c "roslaunch controller control.launch; exec bash;"'