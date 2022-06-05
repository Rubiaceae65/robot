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

WORKDIR /srv


RUN git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git \
 && git clone https://github.com/osrf/gzweb ; cd gzweb ; git checkout gzweb_1.4.1 


WORKDIR /srv/gzweb
RUN bash -c " export GAZEBO_MODEL_PATH=/srv/aws-robomaker-small-house-world/models ; . /user/share/gazebo/setup.sh ; npm run deploy --- -m local"




#RUN bash -c ". /user/share/gazebo/setup.sh ; npm run deploy --- -t"

