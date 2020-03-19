#!/usr/bin/bash

sudo chmod 0666 /dev/ttyUSB0
gnome-terminal -- bash -c "roscore" 
sleep 1
gnome-terminal -- bash -c "rosrun dobot_driver DobotServer /dev/ttyUSB0; bash" 
sleep 1
gnome-terminal -- bash -c "roslaunch astra_camera stereo_s.launch; bash"
sleep 1
gnome-terminal -- bash -c "rosrun dobot_coordinate_transformer coordinate_transformer; bash"
sleep 1
gnome-terminal -- bash -c "rosrun mouse_cursor mouse_cursor; bash"
