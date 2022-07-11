#!/usr/bin/env bash

ros2 run camera_calibration cameracalibrator \
	  --size=5x7 \
	    --square=0.030 \
	    --no-service-check -p chessboard \
	      image:=/ps5eye/right/image_raw camera:=/ps5eye/right/


