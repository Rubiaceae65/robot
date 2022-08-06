#!/usr/bin/env bash

rosdep update
rosdep install --rosdistro foxy --from-paths src/ -r --ignore-src -y
