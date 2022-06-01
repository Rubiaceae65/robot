FROM lala432/ros-base
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
ENV CCACHE_DIR=/ccache
ARG TARGETPLATFORM
ENV IDIR /srv/
ENV TDIR /tmp


RUN mkdir -p $TDIR && mkdir -p /ccache && chown user:user /ccache && chmod a+rwx /ccache && for p in gcc g++ cc c++; do ln -vs /usr/bin/ccache /usr/local/bin/$p;  done
WORKDIR $TDIR


#kinect2

#opencv4 https://github.com/simonernst/iai_kinect2/commit/d4183790c785f5eac0c1ec376a6131fcfd8425b3
#noetic https://github.com/yudhapane/iai_kinect2/commit/7db3900f02e2f1068b080b541129a615b17fff9f
# fp https://github.com/skalldri/iai_kinect2/commit/7a09886115520bb8e17ceefd30bb6ec40c58ef88


RUN git clone https://github.com/OpenKinect/libfreenect2.git \
 && cd libfreenect2/ \
 && mkdir build && cd build \
 && cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$IDIR/freenect2 | tee /var/log/freenect-configure.log \
 &&  make | tee /var/log/freenect-make.log && make install 
# && cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

#mpu9250
#-DCMAKE_INSTALL_PREFIX=$IDIR/RTIMULib2 
RUN git clone https://github.com/RTIMULib/RTIMULib2.git \
  && cd RTIMULib2/RTIMULib \
  && mkdir build && cd build \
  && cmake .. | tee /var/log/imulib-configure.log \
  && make -j4 \
  && make install \
  && ldconfig 


