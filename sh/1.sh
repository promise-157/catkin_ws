#!/bin/bash
sleep 1s
gnome-terminal -x bash -c "echo nvidia | sudo -S uhubctl  -a cycle -d 5 -p 1-4; exec bash"
sleep 10s
gnome-terminal -x bash -c "source $HOME/realsense_ws/devel/setup.bash;roslaunch realsense2_camera rs_camera.launch; exec bash" 
sleep 3s
gnome-terminal -x bash -c "source $HOME/zhuoyi_ws/devel/setup.bash; roslaunch usb_cam usb_cam-test.launch; exec bash"
sleep 3s
gnome-terminal -x bash -c "source $HOME/livox_ws/devel/setup.bash;roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 3s
gnome-terminal  -x bash -c "roslaunch mavros px4.launch; exec bash"

# er jie duan
sleep 5s
gnome-terminal -x bash -c  "cd /home/nvidia/catkin_ws/src/Challege_ROS/object_det/scripts; python3 det.py; exec bash;" 

sleep 3s
gnome-terminal -x bash -c "roslaunch faster_lio rflysim.launch; exec bash;"
sleep 3s
gnome-terminal -x bash -c  "roslaunch ego_planner rflysim.launch; exec bash;"
#sleep 3s
#gnome-terminal -x bash -c  "roslaunch controller controll.launch; exec bash;"


sleep 1s
gnome-terminal --geometry=40x24+1000+600 -- bash -c "rostopic echo /mavros/state; exec bash"
sleep 1s
gnome-terminal --geometry=40x24+1430+600 -- bash -c "rostopic echo /mavros/local_position/pose; exec bash"
