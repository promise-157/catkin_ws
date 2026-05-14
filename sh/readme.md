catkin_make -DCATKIN_WHITELIST_PACKAGES="controller"
find . -type f -exec touch {} \;




rostopic echo /mavros/local_position/pose
rostopic echo /mavros/state

