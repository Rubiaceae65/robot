FROM lala432/ubuntu-vnc:latest
USER root
RUN apt-get update ; apt-get -y upgrade
ADD ros-workspace /home/user/ros-workspace
RUN chown -R user /home/user/ros-workspace
RUN chmod -R a+rw /home/user/ros-workspace
USER user
RUN rosdep update

#build all ros packages
WORKDIR /home/user/ros-workspace
RUN bash ./init-workspace.sh
RUN bash ./build.sh

ENV HOME /home/user
ENV DISPLAY :1
ENV SHELL /bin/bash

EXPOSE 6080
EXPOSE 5900

RUN rosdep update 
#; rosdep fix-permissions
USER root
# put this in ubuntu-vnc later

RUN rm /bin/tini
RUN apt-get install -y tini
ADD docker/startup.sh /startup.sh
ADD docker/supervisor-ros.conf /etc/supervisor/conf.d/supervisor.conf

USER user


WORKDIR /home/user
ENTRYPOINT ["/startup.sh"]
