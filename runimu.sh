#!/usr/bin/env bash

cd /home/user/src/robot2/ros2-workspace
. /opt/ros/foxy/setup.bash
. install/setup.bash

ros2 launch src/mpu9250_python/launch/imu.py
