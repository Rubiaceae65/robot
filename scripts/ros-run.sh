#!/usr/bin/env bash

. /opt/ros/noetic/setup.bash
. $ROS_WS/devel/setup.bash

unset ROS_IPV6

export IP=`/usr/bin/hostname -i`
export ROS_IP="$IP" 
export ROS_HOSTNAME="$IP"
export PYTHONUNBUFFERED=1
echo "==========================================="
echo "= ROS_IP: $ROS_IP"
echo "= ROS_WS: $ROS_WS"
echo "==========================================="



function rlaunch () {
  echo "STARTING: $@"
  
  LD_PRELOAD=/usr/local/lib/bindport.so roslaunch -v --wait $@ 
  #|& tee /proc/1/fd/1

}


case $LAUNCH in
  ssh)

    cat <<EOF > /home/user/.screenrc
caption string "%?%F%{= Bk}%? %C%A %D %d-%m-%Y %{= kB} %t%= %?%F%{= Bk}%:%{= wk}%? %n "
hardstatus alwayslastline
hardstatus string '%{= kG}[ %{G}%H %{g}][%= %{= kw}%?%-Lw%?%{r}(%{W}%n*%f%t%?(%u)%?%{r})%{w}%?%+Lw%?%?%= %{g}][%{B} %d/%m %{W}%c:%s %{g}]'
EOF

    #ROS_MASTER_URI=""
    echo "set -o allexport" >> /etc/bash.bashrc
    env >> /etc/bash.bashrc
    echo "set +o allexport" >> /etc/bash/bashrc

    mkdir -p -m0755 /var/run/sshd
    mkdir -p -m0755 /run/sshd
    mkdir -p /root/.ssh/
    echo "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQC+S3s6NvvRbYuj9US2STwYx3r/ZiysntHtSMnrmOv5bmtgeLlxjQi6Fj2Szyhki2rtL6+znbHpMUa29hoIkeTOWYvfbei5hOHdISdQIpSo/BHZJOB/iC0RUU9+wPT7DzLWmI6vPJU2/3IIB1eIZvu4dNpdpvgfxf3cQTvKnVbuPwB7KGML5UJyXCTrCgjlQaNr+cB7SpuvfqKLbf0DJ6vJnKJVL6/noPk1F1uyN5sLinAV4sMuKuSf8cu3N43o35fdpVrqPOpUGFpSb0Q1OxJHDGZyffO/sMmLDBlxTuty/mk2xEjpDBuVQlA7duhFDYWnosals72iZjGUOkswXBjMC3QSFW3OtBpL/roTQ3t0RcWtYVuuz4TDLc8lQhQrnG7/Yky/T81UThlPGWeSAMrw+er8bTr4P2z8obXOyAy5ZaisN46Hndqqih3pYbxL8t8EhG2j8PnphiKI4kD/Z3nuEtYyua8scdkqUFN2Izu+K7s2xXRp+bXP22kPl87qlcE= user@debian" > /root/.ssh/authorized_keys
    ssh-keygen -A
    /usr/sbin/sshd -e -D -f /scripts/sshd_config
    # -c host_certificate_file -h host_key_file

  ;;
 
  master)
    #ROS_MASTER_URI=""
    roscore
  ;;
 
  move_base)
    rlaunch robot_launch move_base.launch
  ;;

 
  map)
    rlaunch map_server map_server /opt/ros/groovy/stacks/wg_common/willow_maps/willow-sans-whitelab-2010-02-18-0.025.pgm 0.025
    rosrun robot_pose_publisher robot_pose_publisher
  ;;

  capra_web)
    chown -R user /home/user/.npm
    chown -R user /home/user/capra_web_ui
    cd /home/user/capra_web_ui
    su user -c 'npm run start'
  ;;

  capra_description)
    cd $ROS_WS/src/robot_launch
    python3 -m http.server 88
  ;;

  vision | kinect)
    #export LIBVA_DRIVER_NAME=i965
    # rosrun kinect2_bridge kinect2_bridge _fps_limit:=30 _depth_method:=opencl _reg_method:=cpu _max_depth:=0.2 _min_depth:=0.1 _queue_size:=100 _bilateral_filter:=false _edge_aware_filter:=false _worker_threads:=8
    rlaunch robot_launch kinect2_bridge.launch _fps_limit:=25 _depth_method:=opencl _reg_method:=opencl publish_tf:=true
  ;;

  wsbridge)
    rlaunch robot_launch debug.launch
  ;;
 
  gps)
	  rlaunch /scripts/gps.launch
  ;;

  localization)
	  rlaunch /scripts/localization.launch
  ;;

  state_pub)
	  rlaunch /scripts/state_pub.launch
  ;;
 
  imu)
    #pigpiod
	  #rlaunch /scripts/imu.launch
    #cd /home/user/ros-workspace
    #python3 mputest.py
    i2cdetect -y 1
    rlaunch /scripts/mpu9250.launch
  ;;

  motor)
	  rlaunch robot_launch motor.launch
  ;;


  joy)
	  rlaunch robot_launch joy.launch
  ;;

  gzserver)
    Xvfb :2 -screen 0 1600x1200x16  &
    export DISPLAY=:2.0
	  #rlaunch robot_launch gzserver.launch
    rlaunch /scripts/gzserver.launch
  ;;

  web)
    cd /scripts
    pip3 install jinja2
    python3 httpserver.py
  ;;


  gzweb)
    cd /home/user/gzweb
    npm start -p 8001
  ;;

  dev)
    echo "starting xserver etc. etc."
    #rm $ROS_WS/supervisor.d/supervisor-ros.conf
    exec /usr/bin/tini -s -- /usr/bin/supervisord -n -c /scripts/supervisord-vnc.conf
    #rlaunch robot_launch debug.launch
  ;;


  *)
    echo "don't know what to start, exiting"
    exit 123
  ;;
esac
