#!/usr/bin/env bash

for t in `ros2 topic list`
do
	echo "info: $t"
	ros2 topic info $t --verbose
done
