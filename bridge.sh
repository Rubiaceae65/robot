#!/usr/bin/env bash

R2=/home/user/src/robot2/

# Shell B (ROS 1 + ROS 2):
# Source ROS 1 first:
#$R2/ros-workspace/build.sh
. /opt/ros/noetic/setup.bash
source $R2/ros-workspace/devel/setup.bash
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
# Source ROS 2 next:
. /opt/ros/foxy/setup.bash
source ${R2}/ros2-workspace/install/setup.bash
# For example:
# . /opt/ros/dashing/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311


#export FASTRTPS_DEFAULT_PROFILES_FILE=$R2/ros2-workspace/shm_profile.xml 
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>
#export RMW_FASTRTPS_USE_QOS_FROM_XML=1

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
