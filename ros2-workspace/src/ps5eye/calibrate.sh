#!/usr/bin/env bash

ros2 run camera_calibration cameracalibrator \
	  --size=4x6 \
	    --square=0.030 \
	    --no-service-check -p chessboard \
	      image:=/ps5eye/left/image_raw camera:=/ps5eye/left/



	#	left:=/ps5eye/left/image_raw left_camera:=/ps5eye/left \
	#        right:=/ps5eye/right/image_raw right_camera:=/ps5eye/right 
		  
