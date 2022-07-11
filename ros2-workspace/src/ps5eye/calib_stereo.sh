#!/usr/bin/env bash

ros2 run camera_calibration cameracalibrator \
	  --size=5x7 \
	    --square=0.030 \
	    --no-service-check -p chessboard \
		left:=/ps5eye/left/image_raw left_camera:=/ps5eye/left \
	        right:=/ps5eye/right/image_raw right_camera:=/ps5eye/right 
		  
