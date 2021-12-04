#!/usr/bin/env bash

#ROS_MASTER_URI
. /opt/ros/noetic/setup.bash

catkin config
catkin clean -b --yes --all-profiles --deinit
catkin init

set -eux
rosdep install --from-paths src --ignore-src -r -y
catkin build --force-cmake

