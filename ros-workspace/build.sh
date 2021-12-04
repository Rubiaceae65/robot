#!/usr/bin/env bash

#ROS_MASTER_URI
. /opt/ros/noetic/setup.bash

echo "building for $TARGETPLATFORM "

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
du -lh /ccache > /home/user/ccache-before.txt
ls -lhtr ../
ls -lhtr
ls -lhtr  *

echo "we are $UID in groups `groups` pwd $PWD"


#cd src
#catkin build --force-cmake
if [[ $TARGETPLATFORM == "linux/amd64" ]]
then
    catkin build kinect2_bridge kinect2_calibration kinect2_registration kinect2_viewer robot_launch hoverboard_driver teleop_twist_web cartographer_ros gazebo_ros_2Dmap_plugin map2gazebo
fi
if [[ $TARGETPLATFORM == "linux/arm64" ]]
then
  catkin build robot_launch driver_mpu9250 hoverboard_driver i2c_imu sensor_msgs_ext rtimulib_ros
fi

cp -r build build.temp
cp -r devel devel.temp
cp -r install install.temp
du -lh /ccache > /home/user/ccache-after.txt



#catkin_make
#or colcon build
#cd ..
echo "source devel/setup.sh"

