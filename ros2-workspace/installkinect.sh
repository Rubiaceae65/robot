#!/usr/bin/env bash

IDIR=/srv

cd /home/user/

git clone https://github.com/OpenKinect/libfreenect2.git \
 && cd libfreenect2/ \
 && mkdir build && cd build \
 && cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$IDIR/freenect2  \
 &&  make && sudo make install 
