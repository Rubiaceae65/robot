#!/usr/bin/env bash

if ! ls /swapfile
then
  fallocate -l 4G /swapfile
  chmod 600 /swapfile 
  mkswap /swapfile
fi

swapon /swapfile

