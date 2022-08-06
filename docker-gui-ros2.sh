#!/usr/bin/env bash

xhost +

docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host --env="ROS_MASTER_URI=http://192.168.2.8:11311/" \
  osrf/ros:foxy-desktop \
  rqt
