#!/usr/bin/env bash

cd /home/user/src
git clone https://github.com/RTIMULib/RTIMULib2.git

cd RTIMULib2/RTIMULib
mkdir build
cd build
cmake ..
make 
sudo make install
sudo ldconfig
