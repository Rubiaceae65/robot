#!/usr/bin/env bash
# -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2

colcon build --install-base /home/user/src/robot2/ros2-workspace/install/ --symlink-install --packages-select \
  ros2_shared ros2_hoverboard mpu9250_python opencv_cam ps5eye robot2_description ros2_v4l2_camera audio_msgs gst_bridge gst_pipeline rtabmap_ros \
  kinect2_bridge kinect2_calibration kinect2_registration kinect2_viewer
