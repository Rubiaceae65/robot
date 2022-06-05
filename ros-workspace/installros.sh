#!/usr/bin/env bash

echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \

apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

apt-get update \
    && apt-get install -y --force-yes --no-install-recommends \
        net-tools \
        python3-pip python-dev build-essential python-setuptools \
        python3-wstool man-db locate python3-rosdep  ros-noetic-mavlink python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin python3-catkin-tools bash-completion ros-noetic-rosserial-arduino ros-noetic-amcl ros-noetic-move-base spice-html5 jstest-gtk libxcb1-dev libxcb-damage0-dev libxcb-xtest0-dev libxcb-shm0-dev libxcb-util0-dev libxcb-xkb-dev libxcb-xfixes0-dev libgtk2.0-dev libspice-server-dev libspice-protocol-dev libglib2.0-dev libpixman-1-dev libaudit-dev libxcb1 libxcb-damage0 libxcb-xtest0 libxcb-shm0 libxcb-xkb1 libxcb-xfixes0 libgtk2.0-0 libspice-server1 libglib2.0-0 libpixman-1-0 libaudit1 \
ros-noetic-amcl ros-noetic-code-coverage  vim screen htop \
  python3-pigpio python3-colcon-ros wget unzip ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard minicom python3-qwt3d-qt5 libqwtplot3d-qt5-0 python3-pyqtgraph

