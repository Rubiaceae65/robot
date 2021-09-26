#!/usr/bin/env bash

#ROS_MASTER_URI
. /opt/ros/noetic/setup.bash

echo "building for $BUILDPLATFORM "

#rm -r build/hoverboard_driver
# when not initialized
#if ! ls build/.built_by
#then
 catkin config
 catkin init
# catkin clean
 #catkin build --force-cmake
#fi

set -eux
#rosdep install --from-paths src --ignore-src -r -y


#cd src
catkin build --force-cmake


ls -lhtr /ccache > /home/user/ccache.txt

cp -a build build.temp
cp -a devel devel.temp
cp -a install install.temp



#catkin_make
#or colcon build
#cd ..
echo "source devel/setup.sh"

