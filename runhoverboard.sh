#!/usr/bin/env bash

export ROS_IP=192.168.2.8
export ROS_HOSTNAME=192.168.2.8

cd /home/user/src/robot2/ros-workspace

. /opt/ros/noetic/setup.bash
. devel/setup.bash

#env

roslaunch robot_launch hoverboard.launch
