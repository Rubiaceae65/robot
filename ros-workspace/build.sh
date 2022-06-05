#!/usr/bin/env bash

#ROS_MASTER_URI
. /opt/ros/noetic/setup.bash

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
if [ -z ${TARGETPLATFORM+x} ]; 
 then 
	 echo "var is unset"; 
	 TARGETPLATFORM='linux/arm64'
 else 

	 INDOCKER=1
	 echo "in docker, var is set to '$var'"; 
	#rosdep install --from-paths src --ignore-src -r -y
	du -lh /ccache > /home/user/ccache-before.txt
	ls -lhtr ../
	ls -lhtr
	ls -lhtr  *

	echo "we are $UID in groups `groups` pwd $PWD"


fi

echo "building for $TARGETPLATFORM "




#cd src
#catkin build --force-cmake
if [[ $TARGETPLATFORM == "linux/amd64" ]]
then
    catkin build kinect2_bridge kinect2_calibration kinect2_registration kinect2_viewer robot_launch hoverboard_driver teleop_twist_web cartographer_ros gazebo_ros_2Dmap_plugin map2gazebo
fi
if [[ $TARGETPLATFORM == "linux/arm64" ]]
then
  catkin build -j 1 robot_launch driver_mpu9250 hoverboard_driver i2c_imu sensor_msgs_ext rtimulib_ros
fi

if $INDOCKER
then	
	cp -r build build.temp
	cp -r devel devel.temp
	cp -r install install.temp
	du -lh /ccache > /home/user/ccache-after.txt
fi


#catkin_make
#or colcon build
#cd ..
echo "source devel/setup.sh"

