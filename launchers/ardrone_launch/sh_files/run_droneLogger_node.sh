#! /bin/bash

cd $IBVS_STACK
source setup.sh 
roslaunch ${IBVS_STACK}/launchers/ardrone_launch/launch_files/droneLoggerROSModule.launch --wait drone_id_int:="1" drone_id_namespace:="drone1"

