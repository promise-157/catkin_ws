#!/bin/bash

cd /home/nvidia/catkin_ws
pwd

find . -type f -exec touch {} \; 


catkin_make -DCATKIN_WHITELIST_PACKAGES="controller"

