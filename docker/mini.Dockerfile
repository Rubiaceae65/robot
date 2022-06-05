FROM ubuntu:focal
#RUN yes | unminimize
USER root
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
   apt-get update 


ENV ROS_WS=/home/user/ros-workspace
ARG TARGETPLATFORM
ADD --chown=user:user ros-workspace $ROS_WS
##12 6778.7 ERROR: the following packages/stacks could not have their rosdep keys resolved
#12 6778.7 to system dependencies:
#12 6778.7 robaka: Cannot locate rosdep definition for [range_sensor_layer]
#12 6778.7 rosduct: Cannot locate rosdep definition for [python-pydispatcher]
#12 6778.7 ERROR: the following rosdeps failed to install
#12 6778.7   apt: command [apt-get install -y ros-noetic-joint-state-publisher] failed
#12 6778.7   apt: command [apt-get install -y gazebo11] failed
#12 6778.7   apt: command [apt-get install -y ros-noetic-control-toolbox] failed
#12 6778.7   apt: command [apt-get install -y ros-noetic-rosparam-shortcuts] failed
#12 6778.7   apt: command [apt-get install -y ros-noetic-camera-info-manager] failed
#12 6778.7   apt: Failed to detect successful installation of [ros-noetic-joint-state-publisher]
#12 6778.7   apt: Failed to detect successful installation of [ros-noetic-rosparam-shortcuts]
#12 6778.7   pip: command [python3 -m pip install -U ws4py] failed
#12 6778.7   pip: Failed to detect successful installation of [ws4py]
------

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


WORKDIR $ROS_WS
ENTRYPOINT ["/startup.sh"]
 
