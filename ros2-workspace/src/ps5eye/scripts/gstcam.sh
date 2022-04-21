#!/usr/bin/env bash

START=$(($(date +%s%N)/1000000))


# working modes:
# 3840 * 1080 8,30 fps (15 does not work)
# 1920 * 520 60 fps)
# 2560 * 800 60, 30, 15, 8 fps
# 1280 * 373 120 fps

W=3840
H=1080
F=30

W=2560
H=800
F=8



let CROP="$W / 2"

echo $CROP

GST_DEBUG=:4 gst-launch-1.0 --gst-plugin-path=/home/user/src/robot2/ros2-workspace/install/gst_bridge/lib/gst_bridge/ v4l2src extra-controls="c,exposure_auto=0,power_line_frequency=1" ! video/x-raw, height=$H, width=$W, framerate=$F/1 \
     !queue 	! videoconvert  ! queue !  tee name=tee \
	! queue !  videocrop left=$CROP !   rosimagesink ros-name="ps5eye_left" ros-start-time=$START ros-frame-id="ps5eye_left" ros-namespace="ps5eye" ros-topic="left/image_raw" \
	tee. ! queue ! videocrop right=$CROP ! rosimagesink ros-name="ps5eye_right" ros-frame-id="ps5eye_right" ros-start-time=$START ros-namespace="ps5eye" ros-topic="right/image_raw"
