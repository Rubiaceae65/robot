# syntax=docker/dockerfile:1.3
FROM lala432/ros-base:latest
USER root

#ENV HOME /home/user
#ENV DISPLAY :1
#ENV SHELL /bin/bash
#EXPOSE 6080
#EXPOSE 5900


ENV ROS_WS=/home/user/ros-workspace
ARG TARGETPLATFORM

ENV CCACHE_DIR=/ccache
RUN mkdir -p /ccache ; chown user:user /ccache && chmod a+rwx /ccache && for p in gcc g++ cc c++; do ln -vs /usr/bin/ccache /usr/local/bin/$p;  done


ENV BIND_PORT_RANGE=10000-10100
ADD preload/bindport.c /usr/local/lib/
RUN cd /usr/local/lib && gcc -nostartfiles -fpic -shared bindport.c -o bindport.so -ldl -D_GNU_SOURCE


ADD docker/authorized_keys /root/.ssh/authorized_keys
ADD --chown=user:user docker/authorized_keys /home/user/.ssh/authorized_keys
ADD docker/sshd_config /etc/ssh/sshd_config
#RUN mkdir -p /root/.ssh ; mkdir -p /home/user/.ssh ; chown user:user /home/user/.ssh ; chmod a+rwx /home/user/.ssh


ADD docker/supervisor-xserver.conf /etc/supervisor/conf.d/supervisor-xserver.conf


# does not work on arm
RUN apt-get update && apt-get install -y  beignet-dev i965-va-driver vainfo intel-media-va-driver libva-glx2 libva-dev ; true
#might not work on arm
RUN apt-get update && apt-get -y install intel-opencl-icd iputils-ping less ; true

## fixes iai_kinect2 build error (does not work on arm)
RUN ln -s /usr/lib/x86_64-linux-gnu/libOpenCL.so.1 /usr/lib/x86_64-linux-gnu/libOpenCL.so ; true


RUN git clone https://github.com/OpenKinect/libfreenect2.git \
 && cd libfreenect2/ \
 && mkdir build && cd build \
 && cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 | tee /var/log/freenect-configure.log \
 &&  make | tee /var/log/freenect-make.log && make install 
# && cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

RUN git clone https://github.com/RTIMULib/RTIMULib2.git \
  && cd RTIMULib2/RTIMULib \
  && mkdir build && cd build \
  && cmake .. | tee /var/log/imulib-configure.log \
  && make -j4 \
  && make install \
  && ldconfig 

#RUN cd RTIMULib2/python && python setup.py build && python setup.py install 
#might not work on amd64
RUN apt-get update && apt-get -y install linux-firmware-raspi2  ; true
RUN apt-get update && apt-get -y install i2c-tools python3-smbus bluez bluez-tools 
RUN apt-get -y install libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential nodejs npm
USER root
WORKDIR /home/user
RUN git clone https://github.com/osrf/gzweb ; cd gzweb ; git checkout gzweb_1.4.1
WORKDIR /home/user/gzweb
RUN bash -c ". /user/share/gazebo/setup.sh ; npm run deploy --- -m"
#RUN bash -c ". /user/share/gazebo/setup.sh ; npm run deploy --- -t"

RUN pip3 install kubernetes
RUN pip3 install jinja2

USER root
WORKDIR /home/user
RUN git clone https://github.com/clubcapra/capra_web_ui.git
WORKDIR /home/user/capra_web_ui
RUN npm i 
#RUN npm run build

WORKDIR /home/user
RUN wget https://github.com/clubcapra/capra_web_ui/releases/download/v2.3.14/capra_web_ui_setup.deb 
RUN apt-get -y install libsecret-1-0 libnotify4 libappindicator3-1
#FIXME broken on arm$ RUN dpkg -i capra_web_ui_setup.deb
# Copy ros workspace
USER root
WORKDIR /home/user
RUN mkdir -p $ROS_WS 


RUN apt-get update ; apt-get -y install ninja-build stow fuse sshfs libceres-dev lua5.3 liblua5.3-dev ros-noetic-hector-gazebo
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1
RUN apt-get -y install libgmock-dev ros-noetic-tf2-tools ros-noetic-tf2-2d ros-noetic-tf2-bullet ros-noetic-tf2-sensor-msgs ros-noetic-tf2-web-republisher python3-rosgraph python3-std-msgs python3-rosgraph-msgs python3-roslib python3-rospy python3-actionlib python3-geometry-msgs python3-tf2 python3-tf2-ros
RUN apt-get -y install libmng2 qt5-image-formats-plugins ros-noetic-gazebo-ros-control-select-joints ros-noetic-hector-components-description ros-noetic-hector-compressed-map-transport ros-noetic-hector-compressed-map-transport-dbgsym ros-noetic-hector-gazebo-plugins-dbgsym \
  ros-noetic-hector-gazebo-thermal-camera-dbgsym ros-noetic-hector-geotiff ros-noetic-hector-geotiff-dbgsym ros-noetic-hector-geotiff-launch ros-noetic-hector-geotiff-plugins ros-noetic-hector-geotiff-plugins-dbgsym ros-noetic-hector-imu-attitude-to-tf \
  ros-noetic-hector-imu-attitude-to-tf-dbgsym ros-noetic-hector-imu-tools ros-noetic-hector-imu-tools-dbgsym ros-noetic-hector-localization ros-noetic-hector-map-server ros-noetic-hector-map-server-dbgsym ros-noetic-hector-map-tools ros-noetic-hector-mapping \
  ros-noetic-hector-mapping-dbgsym ros-noetic-hector-marker-drawing ros-noetic-hector-models ros-noetic-hector-nav-msgs ros-noetic-hector-pose-estimation ros-noetic-hector-pose-estimation-core ros-noetic-hector-pose-estimation-core-dbgsym ros-noetic-hector-pose-estimation-dbgsym \
  ros-noetic-hector-sensors-description ros-noetic-hector-sensors-gazebo ros-noetic-hector-slam ros-noetic-hector-slam-launch ros-noetic-hector-trajectory-server ros-noetic-hector-trajectory-server-dbgsym ros-noetic-hector-xacro-tools ros-noetic-message-to-tf \
  ros-noetic-velodyne-gazebo-plugins ros-noetic-depthimage-to-laserscan \
  ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3

RUN apt-get -y install libsdformat-dev libgazebo11-dev libgazebo11 

RUN apt -y remove gazebo11 ros-noetic-gazebo-*
RUN apt -y install gazebo11 ros-noetic-gazebo-*


#RUN ls -lhtr /home/user/* ; false
#RUN chown -R user:user /home/user/.ros
RUN --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR chown -R user:user $CCACHE_DIR

RUN apt-get -y install ros-noetic-rtabmap ros-noetic-rtabmap-ros python3-pydispatch

RUN pip3 install ws4py  
ADD --chown=user:user ros-workspace $ROS_WS
#RUN chown user:user $ROS_WS
USER root

WORKDIR $ROS_WS
RUN apt-get -y install python3-rosdep  python3-catkin
RUN rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y

RUN --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR src/cartographer/scripts/install_abseil.sh

RUN chown -R user:user /home/user/.config

RUN --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR chmod -R a+rwx /ccache
RUN --mount=type=cache,mode=0777,uid=1000,id=r_build_$TARGETPLATFORM,target=$ROS_WS/build \
    --mount=type=cache,mode=0777,uid=1000,id=r_install_$TARGETPLATFORM,target=$ROS_WS/install \
    --mount=type=cache,mode=0777,uid=1000,id=r_devel_$TARGETPLATFORM,target=$ROS_WS/devel \
    --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR \
    chown user:user $ROS_WS/build ; chown user:user $ROS_WS/install ; chown user:user $ROS_WS/devel ; chmod -R a+rwx $ROS_WS/build ; chmod -R a+rwx $ROS_WS/install ; chmod -R a+rwx $ROS_WS/devel




#build ros workspace, using a cached version
USER root
WORKDIR $ROS_WS

RUN --mount=type=cache,mode=0777,uid=1000,id=r_build_$TARGETPLATFORM,target=$ROS_WS/build \
    --mount=type=cache,mode=0777,uid=1000,id=r_install_$TARGETPLATFORM,target=$ROS_WS/install \
    --mount=type=cache,mode=0777,uid=1000,id=r_devel_$TARGETPLATFORM,target=$ROS_WS/devel \
    --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR \
    ./build.sh

#RUN ls -lhtr
# those directories where mounted with cached versions, now they are not, but the build script already copied them to .temp ones
RUN rm -r build ; rm -r install ; rm -r devel 
RUN mv build.temp build ; mv install.temp install ; mv devel.temp devel 
#RUN cp -a build.temp build ; cp -a install.temp install ; cp -a devel.temp devel 

#make stuff owned by user (build for some reason does not work as user)
RUN chown user:user $ROS_WS/build ; chown user:user $ROS_WS/install ; chown user:user $ROS_WS/devel ; chmod -R a+rwx $ROS_WS/build ; chmod -R a+rwx $ROS_WS/install ; chmod -R a+rwx $ROS_WS/devel


#RUN rosdep update ; rosdep fix-permissions
ADD docker/startup-new.sh /startup.sh
USER root
ENV DBUS_SESSION_BUS_ADDRESS="unix:path=/var/run/dbus/system_bus_socket"
#USER user
WORKDIR $ROS_WS
ENTRYPOINT ["/startup.sh"]
