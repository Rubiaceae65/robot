#!/usr/bin/env bash

. ~/.profile 
set -eux
GR="`git rev-parse --show-toplevel`"

echo $GR
#kubectl create configmap launchfiles --from-file=$GR/ros-workspace/src/robot_launch/launch/ --dry-run=client -o yaml

create_config () {
  #kubectl create configmap ros-scripts --from-file=$GR/scripts/ --dry-run=client -o yaml | kubectl apply -f -
  kubectl --namespace="$1" create configmap ros-scripts --from-file=$GR/scripts/ --dry-run=client -o yaml | kubectl apply -f -

  #kubectl create configmap launchfiles --from-file=$GR/ros-workspace/src/robot_launch/launch/ --dry-run=client -o yaml | kubectl apply -f -
  kubectl --namespace="$1" create configmap launchfiles --from-file=$GR/ros-workspace/src/robot_launch/launch/ --dry-run=client -o yaml | kubectl apply -f -
}

create_config simul
create_config robot
