#!/usr/bin/env bash

set -eux
. ~/.profile
#helm install --set simulation.enabled=true simulation . -n simul
#helm upgrade --set simulation.enabled=true simulation . -n simul
GR="`git rev-parse --show-toplevel`"

HD="$GR/k8/helm"

#kubectl taint --overwrite node k3os-341 node-role.kubernetes.io/master:NoSchedule

restartall () {

  deploys=`kubectl get deployments -n $1 | grep -v NAME | grep -v master | grep -v ssh | grep -v xserver| cut -d ' ' -f 1`
  for deploy in $deploys; do
    kubectl rollout restart deployments/$deploy -n $1
  done

}

#helm uninstall robot2

#if ! helm install robot2-test --set 'simulation.enabled=true' . -n robottestblaat
#then
#  helm upgrade robot2-test --set 'simulation.enabled=true' . -n robottestblaat 
#fi

$GR/k8/configmap.sh
if ! helm install robot2-hw --set 'simulation.enabled=false' $HD -n robot 
then
  helm upgrade robot2-hw --set 'simulation.enabled=false' $HD -n robot
#  restartall robot
fi

if true
then
  if ! helm install robot-simul --set 'simulation.enabled=true' $HD -n simul
  then
    helm upgrade robot-simul --set 'simulation.enabled=true' $HD -n simul
#    restartall simul
  fi
fi

