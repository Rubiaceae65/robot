#!/bin/bash

sigint_handler()
{
  kill $PID
  exit
}

trap sigint_handler SIGINT
 
if [ -z $WATCHES ]
then
	WATCHES="$1"
fi

while true; do
	LS=`ls --full-time $WATCHES`
	OLDLS="$LS"


  $@ &
  PID=$!
 
  #inotifywait -e modify -e move -e create -e delete -e attrib -r `pwd`
  # no inotify on sshfs  
  while [[ "$LS" == "$OLDLS" ]]
    do
	OLDLS="$LS"
	LS=`ls --full-time $WATCHES`
	#echo $OLDLS
	#echo $LS
	sleep 1
   done
echo "files changed"
  kill $PID
  sleep 1
done
