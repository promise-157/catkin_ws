#!/bin/bash
sleep 1s
gnome-terminal -x bash -c "echo nvidia | sudo -S uhubctl  -a cycle -d 5 -p 1-4; exec bash"
sleep 15s
gnome-terminal -x bash -c "source $HOME/realsense_ws/devel/setup.bash;roslaunch realsense2_camera rs_camera.launch; exec bash" 
sleep 3s
gnome-terminal -x bash -c "source $HOME/zhuoyi_ws/devel/setup.bash; roslaunch usb_cam usb_cam-test.launch; exec bash"
sleep 3s


# er jie duan
sleep 5s
gnome-terminal -x bash -c  "cd /home/nvidia/catkin_ws/src/Challege_ROS/recognize_aruco; python3 image.py; exec bash;" 

