#!/usr/bin/env bash

set -eux
. ~/.profile
function r () {
  TYPE=$1
  FILE=$2.yaml
  NAME=$2

    if [[ "$TYPE" == "statefulset" ]]
    then
      kubectl delete statefulset $NAME
    fi

  kubectl apply -f $FILE
  #kubectl rollout restart $TYPE/$NAME

  ATTEMPTS=0
  ROLLOUT_STATUS_CMD="kubectl rollout status $TYPE/$NAME -n default"
  until $ROLLOUT_STATUS_CMD || [ $ATTEMPTS -eq 60 ]; do
    $ROLLOUT_STATUS_CMD
    ATTEMPTS=$((attempts + 1))
    sleep 10
  done
  #--tail=100 
  kubectl logs -l app=$NAME -f --since=1h --pod-running-timeout=1m

}
function b () {
  # --progress plain
  docker buildx build   \
    --push \
    -f docker/$1.Dockerfile \
    --platform linux/arm64/v8,linux/amd64 \
    --tag lala432/$1:latest $2
#  docker build -t lala432/$1:latest $2
#  docker build -t $1:latest $2
#  docker push lala432/$1:latest
}
b ubuntu-cross ./
#b ubuntu-ros-desktop ./
#b ubuntu-gnome ubuntu-gnome
#b ubuntu-gnome-vgl ubuntu-gnome-vgl
#b ubuntu-gnome-xrdp ubuntu-gnome-xrdp
#b ubuntu-gnome-xrdp ubuntu-gnome-spice

#r statefulset vdesk

#docker build -t ubuntu-gnome-xrdp:18.04 ubuntu-gnome-xrdp

