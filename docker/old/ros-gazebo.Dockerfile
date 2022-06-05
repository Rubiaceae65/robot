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

RUN apt-get update && apt-get install -y  beignet-dev i965-va-driver vainfo intel-media-va-driver libva-glx2 libva-dev ; true
#might not work on arm
RUN apt-get update && apt-get -y install intel-opencl-icd iputils-ping less ; true

RUN apt-get -y install libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential nodejs npm
USER root
WORKDIR /srv/
RUN git clone https://github.com/osrf/gzweb ; cd gzweb ; git checkout gzweb_1.4.1
#WORKDIR /srv/gzweb
RUN bash -c "cd /srv/gzweb ; . /user/share/gazebo/setup.sh ; npm run deploy --- -m"
#
