roslaunch kr_mav_launch track_circle.launch & sleep 5;
roslaunch px4ctrl run_mavros_gazebo.launch & sleep 5;
rosrun rqt_reconfigure rqt_reconfigure 

