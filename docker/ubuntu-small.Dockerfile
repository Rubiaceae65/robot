# syntax=docker/dockerfile:1.3

# use xx-apt for packages
# use cache for ccache and ros build per architecture

# https://github.com/docker/buildx/issues/156
# https://github.com/docker/buildx/issues/255
# https://github.com/docker/buildx/issues/156 # move cache volumes between hosts
#https://www.docker.com/blog/multi-platform-docker-builds/
# rsync cache trick: http://erouault.blogspot.com/2019/05/incremental-docker-builds-using-ccache.html

#FROM --platform=$TARGETPLATFORM tonistiigi/xx AS xx
FROM lala432/ubuntu-vnc:latest
USER root
#ENV ROS_WS=/workspaces/robot2/ros-workspace
ENV ROS_WS=/home/user/ros-workspace


ENV CCACHE_DIR=/ccache

ARG TARGETPLATFORM
RUN apt-get update ; apt-get -y upgrade ; apt-get install -y ccache distcc openssh-client openssh-server ros-noetic-rosbridge-server
RUN rm /bin/tini
RUN apt-get install -y tini
ADD docker/startup.sh /startup.sh
ADD preload/bindport.c /usr/local/lib/
RUN cd /usr/local/lib ; gcc -nostartfiles -fpic -shared bindport.c -o bindport.so -ldl -D_GNU_SOURCE
ENV BIND_PORT_RANGE=10000-10100
ADD docker/supervisord.conf /etc/supervisor/supervisord.conf
ADD docker/supervisor-ros.conf /etc/supervisor/conf.d/supervisord.conf
#   Your libOpenCL.so is incompatible with CL/cl.h.  Install ocl-icd-opencl-dev
 
RUN apt-get install -y ros-noetic-libuvc-camera ros-noetic-openni2-camera ros-noetic-depth-image-proc ros-noetic-camera-info-manager ros-noetic-camera-calibration ros-noetic-image-pipeline ros-noetic-image-rotate libfreenect-bin libfreenect-demos libfreenect-dev libfreenect-doc libfreenect0.5 build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev libva-dev ros-noetic-rtabmap-ros

# does not work on arm
RUN  apt-get install -y  beignet-dev i965-va-driver vainfo intel-media-va-driver libva-glx2 libva-dev ; true
## fixes iai_kinect2 build error (does not work on arm)
RUN ln -s /usr/lib/x86_64-linux-gnu/libOpenCL.so.1 /usr/lib/x86_64-linux-gnu/libOpenCL.so ; true

RUN git clone https://github.com/OpenKinect/libfreenect2.git \
 && cd libfreenect2/ \
 && mkdir build && cd build \
 && cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 | tee /var/log/freenect-configure.log \
 &&  make | tee /var/log/freenect-make.log && make install 
# && cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

RUN mkdir -p $ROS_WS ; mkdir -p /root/.ssh ; mkdir -p /home/user/.ssh ; chown user:user /home/user/.ssh ; chmod a+rwx /home/user/.ssh
ADD docker/authorized_keys /root/.ssh/authorized_keys
ADD --chown=user:user docker/authorized_keys /home/user/.ssh/authorized_keys
ADD docker/sshd_config /etc/ssh/sshd_config

RUN mkdir -p /ccache ; chown user:user /ccache ; chmod a+rwx /ccache
ADD --chown=user:user ros-workspace $ROS_WS
RUN chown user:user $ROS_WS
#RUN chown -R user /home/user/ros-workspace
#RUN chmod -R a+rw /home/user/ros-workspace

RUN for p in gcc g++ cc c++; do ln -vs /usr/bin/ccache /usr/local/bin/$p;  done


USER user
#RUN rosdep update

# use ccache (make it appear in path earlier then /usr/bin/gcc etc)

#RUN --mount=type=cache,target=/ccache/ ccache -s
#RUN --mount=type=cache,id=$BUILDPLATFORM,target=/ccache/

#build all ros packages
WORKDIR $ROS_WS
#RUN bash ./init-workspace.sh
RUN printf "hello \n asdf \n asdf $TARGETPLATFORM"
RUN echo $TARGETPLATFORM
RUN --mount=type=cache,mode=0777,uid=1000,id=r_build_$TARGETPLATFORM,target=$ROS_WS/build \
    --mount=type=cache,mode=0777,uid=1000,id=r_install_$TARGETPLATFORM,target=$ROS_WS/install \
    --mount=type=cache,mode=0777,uid=1000,id=r_devel_$TARGETPLATFORM,target=$ROS_WS/devel \
    --mount=type=cache,mode=0777,uid=1000,id=ccache_$TARGETPLATFORM,target=$CCACHE_DIR \
    ./build.sh

RUN ls -lhtr
RUN rm -r build ; rm -r install ; rm -r devel 
RUN cp -a build.temp build ; cp -a install.temp install ; cp -a devel.temp devel 

ENV HOME /home/user
ENV DISPLAY :1
ENV SHELL /bin/bash

EXPOSE 6080
EXPOSE 5900

#RUN rosdep update 
#; rosdep fix-permissions
USER root
# put this in ubuntu-vnc later
ADD docker/bash.bashrc /etc/bash.bashrc
# give access to /dev/dri for kinect
RUN usermod -aG render user ; usermod -aG kvm user
USER user



WORKDIR $ROS_WS
ENTRYPOINT ["/startup.sh"]
