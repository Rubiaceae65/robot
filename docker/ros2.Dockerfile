FROM lala432/mini-withbase
#RUN yes | unminimize
USER root
ENV DEBIAN_FRONTEND noninteractive
ENV HOME /home
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO foxy
ENV HOME /home/user
ENV DISPLAY :1
ENV SHELL /bin/bash

ENV CCACHE_DIR=/ccache

RUN mkdir -p /home/user/src/robot2 ; chown user:user /home/user/src/robot2
ENV ROS_WS=/home/user/src/robot2/ros2-workspace
ARG TARGETPLATFORM
ADD --chown=user:user ros2-workspace $ROS_WS

# clear ccache (so that later when image is build, user can write to it
RUN rm -r /ccache && mkdir /ccache && chmod a+rwx /ccache

#RUN apt-get -y install python3-rosdep  python3-catkin
#RUN rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y

#RUN --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR src/cartographer/scripts/install_abseil.sh

#RUN chown -R user:user /home/user/.config
#RUN mkdir -p /ccache ; chown user:user /ccache && chmod a+rwx /ccache && for p in gcc g++ cc c++; do ln -vs /usr/bin/ccache /usr/local/bin/$p;  done


#RUN --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR chmod -R a+rwx /ccache


#RUN --mount=type=cache,mode=0777,uid=1000,id=r_build_$TARGETPLATFORM,target=$ROS_WS/build \
#    --mount=type=cache,mode=0777,uid=1000,id=r_install_$TARGETPLATFORM,target=$ROS_WS/install \
#    --mount=type=cache,mode=0777,uid=1000,id=r_devel_$TARGETPLATFORM,target=$ROS_WS/devel \
#    --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR \
#    chown user:user $ROS_WS/build ; chown user:user $ROS_WS/install ; chown user:user $ROS_WS/devel ; chmod -R a+rwx $ROS_WS/build ; chmod -R a+rwx $ROS_WS/install ; chmod -R a+rwx $ROS_WS/devel




#build ros workspace, using a cached version
WORKDIR $ROS_WS
RUN ./installros2.sh
RUN ./rosdep.sh
RUN sudo apt install -y python3-colcon-common-extensions


#RUN --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR \
#    ./build.sh

USER user
RUN --mount=type=cache,mode=0777,uid=1000,id=r2_build2_$TARGETPLATFORM,target=$ROS_WS/build \
    --mount=type=cache,mode=0777,uid=1000,id=r2_install2_$TARGETPLATFORM,target=$ROS_WS/install \
    --mount=type=cache,mode=0777,uid=1000,id=r2_devel2_$TARGETPLATFORM,target=$ROS_WS/devel \
    --mount=type=cache,mode=0777,uid=1000,id=r2_ccache2_$TARGETPLATFORM,target=$CCACHE_DIR \
    ./build.sh

#RUN ls -lhtr
# those directories where mounted with cached versions, now they are not, but the build script already copied them to .temp ones
RUN rm -r build ; rm -r install ; rm -r devel 
RUN mv build.temp build ; mv install.temp install ; mv devel.temp devel 

#make stuff owned by user (build for some reason does not work as user)
#RUN chown user:user $ROS_WS/build ; chown user:user $ROS_WS/install ; chown user:user $ROS_WS/devel ; chmod -R a+rwx $ROS_WS/build ; chmod -R a+rwx $ROS_WS/install ; chmod -R a+rwx $ROS_WS/devel
USER root
RUN mkdir -p /workspaces ; chmod a+rwx /workspaces
ADD docker/startup.sh /startup.sh

USER user
WORKDIR $ROS_WS
ENTRYPOINT ["/startup.sh"]
 
