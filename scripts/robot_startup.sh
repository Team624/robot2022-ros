#! /bin/bash
source /opt/ros/melodic/setup.bash
source ~/robot2022-ros/devel/setup.bash

# Start roscore and wait till its finished
roscore -p 11311 &
sleep 5

roslaunch robot_launch robot_startup.launch
