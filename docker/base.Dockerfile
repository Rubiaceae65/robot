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
    apt -y clean && rm -rf /var/lib/apt/lists/*

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update \
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
git &&  apt -y clean && rm -rf /var/lib/apt/lists/* 


ADD docker/startup.sh /
ADD docker/supervisord.conf /etc/supervisord.conf
ADD docker/screenrc /home/user/.screenrc
ADD docker/bash.bashrc /etc/bash.bashrc

RUN chmod a+rwx /var/log/ && \
    mkdir -p /var/log/supervisor && chmod a+rwx /var/log/supervisor && \
    mkdir -p /var/run && chmod a+rwx /var/run && \
    groupadd -g 1000 user && \
    useradd -g 1000 -u 1000  user && \
    mkdir -p /run && chown user:user /run && \
    chown -R user:user /home/user && chmod -R a+rwx /home/user && \
    mkdir /home/.vscode-server && chown -R user:user /home/.vscode-server && chmod -R a+rwx /home/.vscode-server && \
    mkdir /home/user/.vscode-server && chown -R user:user /home/user/.vscode-server && chmod -R a+rwx /home/user/.vscode-server && \
    mkdir -p /var/okteto/remote && chown -R user:user /var/okteto


#RUN chmod -R a+rwx /home
#RUN mkdir /home/ubuntu
#RUN mkdir /home/ubuntu/.vscode-server
#RUN chown user:user /home/ubuntu
#RUN chmod a+rwx /home/ubuntu


#RUN apt-get -y install git
#RUN git clone https://gitlab.com/spice/x11spice.git
#RUN cd x11spice ; ./autogen.sh ; ./configure ; make ; make install


RUN mkdir pigpiosrc && cd pigpiosrc && wget https://github.com/joan2937/pigpio/archive/master.zip && unzip master.zip && cd pigpio-master && make && make install

#/dev/input/js0
#joystick ubuntu
# give access to devices
#FIXME???? RUN mkdir /dev/input
RUN usermod -aG input user && \
    usermod -aG dialout user && \
    usermod -aG messagebus user && \
    usermod -aG video user && \
    usermod -aG render user && \
    usermod -aG kvm user && \
    usermod -aG backup user && \
    usermod -aG root user # FIXME: /dev/bus/usb has group root
    

#RUN usermod -aG sudo user
#RUN rosdep init
#RUN rosdep update
#EXPOSE 6080
#EXPOSE 5900
#RUN rosdep update 
#; rosdep fix-permissions

USER user
WORKDIR /home/user
ENTRYPOINT ["/startup.sh"]
