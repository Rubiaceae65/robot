#!/usr/bin/env bash
#
# 
#

set -eux
. ~/.profile
PLATFORM="linux/arm64,linux/amd64"
DEVPLATFORM="linux/amd64"
PREFIX=lala432
WAIT=0
# use multiarch build (uses qemu-user-static)
MA=0

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

if [ $MA -eq 1 ]
then
  kubectl apply -f k8/qemu-user-static.yml
fi


function r () {
  TYPE=$1
  NAME=$2
  FILE=$3
  #$2.yaml

    if [[ "$TYPE" == "statefulset" ]]
    then
      kubectl delete statefulset $NAME ; true
    fi
    if [[ "$TYPE" == "daemonset" ]]
    then
      kubectl delete daemonset $NAME ; true
    fi


  kubectl apply -f $FILE
  if [[ "$TYPE" == "deployment" ]]
  then
      kubectl rollout restart $TYPE/$NAME
  fi


  #kubectl rollout restart $TYPE/$NAME
  if [ $WAIT -eq 1 ]
  then
    ATTEMPTS=0
    ROLLOUT_STATUS_CMD="kubectl rollout status $TYPE/$NAME -n default"
    until $ROLLOUT_STATUS_CMD || [ $ATTEMPTS -eq 60 ]; do
      $ROLLOUT_STATUS_CMD
      ATTEMPTS=$((attempts + 1))
      sleep 10
    done
  fi
  #--tail=100 
  #kubectl logs -l app=$NAME -f --since=1h --pod-running-timeout=1m

}
function b () {
  NAME=$1
  DIR=$2
  DOCKERFILE=$3 
  # --progress plain
  if [ $MA -eq 0 ]
  then
    echo "building only amd64"
    #dev (load is super slow)
    docker buildx build   \
      --push \
      -f $DOCKERFILE \
      --platform $DEVPLATFORM \
      --cache-from=$PREFIX/$NAME:cache \
      --cache-to=$PREFIX/$NAME:cache \
      --tag $PREFIX/$NAME:latest $DIR
     
  docker pull $PREFIX/$NAME:latest
  else
    # multiarch
    docker buildx build   \
      --push \
      -f $DOCKERFILE --platform $PLATFORM --tag $PREFIX/$NAME:latest $DIR
#--cache-from=$PREFIX/$NAME:cache  --cache-to=$PREFIX/$NAME:cache 
    #docker pull $PREFIX/$NAME:latest

  fi


#  docker build -t lala432/$1:latest $2
#  docker build -t $1:latest $2
#  docker push lala432/$1:latest
}

#b k8s-host-device-plugin k8/k8s-host-device-plugin/ k8/k8s-host-device-plugin/Dockerfile
#b ros-base ./ docker/base.Dockerfile
b ros-dev ./ docker/ros-dev.Dockerfile

r deployment ros-pi k8/ros-pi.yaml
r deployment ros-6500 k8/ros-6500.yaml

#b ubuntu-cross ./ docker/ubuntu-cross.Dockerfile
#r daemonset host-device-plugin-daemonset k8/ib-device-plugin.yaml 

#b ubuntu-ros-desktop ./
#b ubuntu-gnome ubuntu-gnome
#b ubuntu-gnome-vgl ubuntu-gnome-vgl
#b ubuntu-gnome-xrdp ubuntu-gnome-xrdp
#b ubuntu-gnome-xrdp ubuntu-gnome-spice

#r statefulset vdesk

#docker build -t ubuntu-gnome-xrdp:18.04 ubuntu-gnome-xrdp

