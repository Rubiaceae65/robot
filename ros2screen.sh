#!/usr/bin/env bash

cd /home/user/src/robot2/ros2-workspace
. /opt/ros/foxy/setup.bash
. install/setup.bash
. env.sh
screen -S ros2-$1
