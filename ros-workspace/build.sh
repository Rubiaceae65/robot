#!/usr/bin/env bash

#ROS_MASTER_URI
. /opt/ros/noetic/setup.bash

echo "building for $TARGETPLATFORM "


#CMAKE_ARGS='-Dfreenect2_DIR=/srv/freenect2/lib/cmake/freenect2 -DCMAKE_BUILD_TYPE="Release"'
CMAKE_ARGS='-Dfreenect2_DIR=/srv/freenect2/lib/cmake/freenect2"'
#CMAKE_ARGS=''

export CMAKE_PREFIX_PATH=/srv/RTIMULib2:/srv/freenect2:$CMAKE_PREFIX_PATH


#rm -r build/hoverboard_driver
# when not initialized
#if ! ls build/.built_by
#then
 catkin config
 catkin init
 catkin clean
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
  catkin build --cmake-args="\'${CMAKE_ARGS}\'"  kinect2_bridge kinect2_calibration kinect2_registration kinect2_viewer
#  catkin build kinect2_bridge kinect2_calibration kinect2_registration kinect2_viewer robot_launch hoverboard_driver teleop_twist_web 



  #map2gazebo move_base_flex rtab_dumpster aws_robomaker_small_warehouse_world
 
  #catkin build kinect2_bridge kinect2_calibration kinect2_registration kinect2_viewer robot_launch hoverboard_driver teleop_twist_web cartographer_ros map2gazebo move_base_flex rtab_dumpster aws_robomaker_small_warehouse_world
    #gazebo_ros_2Dmap_plugin 
fi
if [[ $TARGETPLATFORM == "linux/arm64" ]]
then
  catkin build robot_launch hoverboard_driver i2c_imu sensor_msgs_ext rtimulib_ros
fi
#driver_mpu9250 
cp -r build build.temp
cp -r devel devel.temp
cp -r install install.temp
du -lh /ccache > /home/user/ccache-after.txt



#catkin_make
#or colcon build
#cd ..
echo "source devel/setup.sh"

