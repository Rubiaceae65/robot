#!/bin/bash

#ROS_MASTER_PORT=tcp://10.43.0.61:11311

export IP=`/usr/bin/hostname -i`
export ROS_IP="$IP" 
export ROS_HOSTNAME="$IP"
#export ROS_MASTER_URI=http://$IP:11311 \
echo STARTING
echo $PATH
echo "my ros ip is: $ROS_IP"

exec /usr/bin/tini -s -- /usr/bin/supervisord -n
