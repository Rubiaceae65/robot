#!/usr/bin/env bash

cd /home/user/src/robot2/ros2-workspace
. ../env.sh
. /opt/ros/foxy/setup.bash
. install/setup.bash

#ros2 launch src/mpu9250_python/launch/imu.py
ros2 run rosboard rosboard_node
