#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Ready to Fly, Please Away From it "
  #exit 1
fi

read -p "Press [Enter] to go to [3.5, 0, 2]"
rosservice call /mav_services/goTo "goal: [3.5, 0.0, 2, 0.0]"
sleep 1

read -p "Press [Enter] to circle track {Ax: 3, Ay: 3, T: 9.0, duration: 12.0}"
rosservice call /mav_services/circle "{Ax: 3, Ay: 3, T: 9.0, duration: 12.0}" 
sleep 1
