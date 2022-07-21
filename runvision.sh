#!/usr/bin/env bash

cd /home/user/src/robot2/ros2-workspace
. /opt/ros/foxy/setup.bash
. install/setup.bash

ros2 launch src/ps5eye/launch/disparity.py
