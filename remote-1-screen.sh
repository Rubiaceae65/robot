#!/usr/bin/env bash

. /opt/ros/noetic/setup.bash

cd /home/user/src/robot2/ros-workspace

. devel/setup.bash

export ROS_MASTER_URI=http://192.168.2.8:11311
screen -S ros_remotge
