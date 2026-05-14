#!/bin/bash

gnome-terminal --geometry=40x24+50+300 -- bash -c "rostopic echo /mavros/state; exec bash"
sleep 1s
gnome-terminal --geometry=40x24+480+310 -- bash -c "rostopic echo /mavros/local_position/pose; exec bash"

