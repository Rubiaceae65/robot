FROM ubuntu:focal
#RUN yes | unminimize

ENV DEBIAN_FRONTEND noninteractive
ENV HOME /home
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENV HOME /home/user
ENV DISPLAY :1
ENV SHELL /bin/bash

# setup timezone
RUN echo 'Europe/Amsterdam' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Europe/Amsterdam /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata gnupg ca-certificates && \
    apt -y clean && rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
   apt-get update \
    && apt-get install -y --force-yes --no-install-recommends supervisor \
        pwgen sudo vim-tiny x11vnc \
        net-tools \
        lxde x11vnc xvfb \
        gtk2-engines-murrine ttf-ubuntu-font-family \
        libreoffice firefox \
        fonts-wqy-microhei \
        nginx \
        python3-pip python-dev build-essential python-setuptools \
        mesa-utils libgl1-mesa-dri \
        python3-wstool man-db locate python3-rosdep  ros-noetic-mavlink python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin python3-catkin-tools bash-completion ros-noetic-rosserial-arduino ros-noetic-amcl ros-noetic-move-base spice-html5 jstest-gtk libxcb1-dev libxcb-damage0-dev libxcb-xtest0-dev libxcb-shm0-dev libxcb-util0-dev libxcb-xkb-dev libxcb-xfixes0-dev libgtk2.0-dev libspice-server-dev libspice-protocol-dev libglib2.0-dev libpixman-1-dev libaudit-dev libxcb1 libxcb-damage0 libxcb-xtest0 libxcb-shm0 libxcb-xkb1 libxcb-xfixes0 libgtk2.0-0 libspice-server1 libglib2.0-0 libpixman-1-0 libaudit1 \
ros-noetic-amcl ros-noetic-code-coverage ros-noetic-desktop-full ros-noetic-dwa-local-planner ros-noetic-eigen-stl-containers ros-noetic-eigenpy ros-noetic-fcl ros-noetic-franka-description ros-noetic-global-planner ros-noetic-gmapping ros-noetic-graph-msgs ros-noetic-imu-filter-madgwick ros-noetic-joy-teleop ros-noetic-joy ros-noetic-map-server ros-noetic-mavlink ros-noetic-move-base ros-noetic-nmea-navsat-driver ros-noetic-object-recognition-msgs ros-noetic-octomap-msgs ros-noetic-octomap ros-noetic-ompl ros-noetic-pybind11-catkin ros-noetic-random-numbers ros-noetic-realsense2-camera ros-noetic-robot-localization ros-noetic-rosparam-shortcuts ros-noetic-rosserial-arduino ros-noetic-rviz-imu-plugin ros-noetic-rviz ros-noetic-spacenav-node ros-noetic-srdfdom ros-noetic-teb-local-planner ros-noetic-teleop-twist-keyboard ros-noetic-twist-mux ros-noetic-warehouse-ros \
    ros-noetic-desktop-full vim screen htop \
  python3-pigpio python3-colcon-ros wget unzip ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard minicom python3-qwt3d-qt5 libqwtplot3d-qt5-0 python3-pyqtgraph xterm tini \
ros-noetic-libuvc-camera ros-noetic-openni2-camera ros-noetic-depth-image-proc ros-noetic-camera-info-manager ros-noetic-camera-calibration ros-noetic-image-pipeline ros-noetic-image-rotate libfreenect-bin libfreenect-demos libfreenect-dev libfreenect-doc libfreenect0.5 build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev libva-dev ros-noetic-rtabmap-ros \
ccache distcc openssh-client openssh-server ros-noetic-rosbridge-server \
ninja-build stow fuse sshfs libceres-dev lua5.3 liblua5.3-dev ros-noetic-hector-gazebo \
libgmock-dev ros-noetic-tf2-tools ros-noetic-tf2-2d ros-noetic-tf2-bullet ros-noetic-tf2-sensor-msgs ros-noetic-tf2-web-republisher python3-rosgraph python3-std-msgs python3-rosgraph-msgs python3-roslib python3-rospy python3-actionlib python3-geometry-msgs python3-tf2 python3-tf2-ros \
 libmng2 qt5-image-formats-plugins ros-noetic-gazebo-ros-control-select-joints ros-noetic-hector-components-description ros-noetic-hector-compressed-map-transport ros-noetic-hector-compressed-map-transport-dbgsym ros-noetic-hector-gazebo-plugins-dbgsym \
  ros-noetic-hector-gazebo-thermal-camera-dbgsym ros-noetic-hector-geotiff ros-noetic-hector-geotiff-dbgsym ros-noetic-hector-geotiff-launch ros-noetic-hector-geotiff-plugins ros-noetic-hector-geotiff-plugins-dbgsym ros-noetic-hector-imu-attitude-to-tf \
  ros-noetic-hector-imu-attitude-to-tf-dbgsym ros-noetic-hector-imu-tools ros-noetic-hector-imu-tools-dbgsym ros-noetic-hector-localization ros-noetic-hector-map-server ros-noetic-hector-map-server-dbgsym ros-noetic-hector-map-tools ros-noetic-hector-mapping \
  ros-noetic-hector-mapping-dbgsym ros-noetic-hector-marker-drawing ros-noetic-hector-models ros-noetic-hector-nav-msgs ros-noetic-hector-pose-estimation ros-noetic-hector-pose-estimation-core ros-noetic-hector-pose-estimation-core-dbgsym ros-noetic-hector-pose-estimation-dbgsym \
  ros-noetic-hector-sensors-description ros-noetic-hector-sensors-gazebo ros-noetic-hector-slam ros-noetic-hector-slam-launch ros-noetic-hector-trajectory-server ros-noetic-hector-trajectory-server-dbgsym ros-noetic-hector-xacro-tools ros-noetic-message-to-tf \
  ros-noetic-velodyne-gazebo-plugins ros-noetic-depthimage-to-laserscan \
  ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3 \
  i2c-tools python3-smbus bluez bluez-tools \
  libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential nodejs npm \
  libsecret-1-0 libnotify4 libappindicator3-1 \
  ros-noetic-rtabmap ros-noetic-rtabmap-ros python3-pydispatch \
 iputils-ping less \
 gazebo11 ros-noetic-gazebo-* \
  libsdformat-dev libgazebo11-dev libgazebo11  \
 git \
  && apt -y clean && rm -rf /var/lib/apt/lists/*  \
  && pip3 install kubernetes jinja2 ws4py \
  && update-alternatives --install /usr/bin/python python /usr/bin/python3 1 \
  && chmod a+rwx /var/log/ && \
    mkdir -p /var/log/supervisor && chmod a+rwx /var/log/supervisor && \
    mkdir -p /var/run && chmod a+rwx /var/run && \
    groupadd -g 1000 user && \
    useradd -g 1000 -u 1000  user && \
    mkdir -p /run && chown user:user /run && \
    chown -R user:user /home/user && chmod -R a+rwx /home/user && \
    mkdir /home/.vscode-server && chown -R user:user /home/.vscode-server && chmod -R a+rwx /home/.vscode-server && \
    mkdir /home/user/.vscode-server && chown -R user:user /home/user/.vscode-server && chmod -R a+rwx /home/user/.vscode-server && \
    mkdir -p /var/okteto/remote && chown -R user:user /var/okteto && \
    usermod -aG input user && \
    usermod -aG dialout user && \
    usermod -aG messagebus user && \
    usermod -aG video user && \
    usermod -aG render user && \
    usermod -aG kvm user && \
    usermod -aG backup user && \
    usermod -aG root user # FIXME: /dev/bus/usb has group root
 
ADD screenrc /home/user/.screenrc
ADD bash.bashrc /etc/bash.bashrc

   
RUN apt-get update && apt-get install -y  beignet-dev i965-va-driver vainfo intel-media-va-driver libva-glx2 libva-dev intel-opencl-icd || true  && apt -y clean && rm -rf /var/lib/apt/lists/*  \


USER user
WORKDIR /home/user
ENTRYPOINT ["/startup.sh"]
