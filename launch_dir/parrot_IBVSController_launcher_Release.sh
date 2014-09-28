#!/bin/bash

# Load ROS_WORKSPACE, etc
. ${IBVS_STACK}/setup.sh

# Load some local variables
NUMID_DRONE=0
NETWORK_ROSCORE=localhost
export ROS_MASTER_URI=http://localhost:11311
OPEN_ROSCORE=1

gnome-terminal	 		\
	--tab --title "roscore" 	--command "bash -c \"
						roscore; 
						exec bash\""  \
	--tab --title "droneLogger"	--command "bash -c \"
						roslaunch parrotLogger parrotLogger.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK}; 
						exec bash\""  \
	--tab --title "ardrone_driver"	--command "bash -c \"
						roslaunch ${IBVS_STACK}/launch_dir/ardrone_launchfiles/ardrone_outdoors.launch --wait drone_id_namespace:=drone$NUMID_DRONE;
						exec bash\""  \
	--tab --title "OpenTLD"		--command "bash -c \"
						env sleep 20s ; 
						roslaunch ${IBVS_STACK}/launch_dir/IBVS2013_launchfiles/opentld_for_IBVSController.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK}; 
						exec bash\""  \
	--tab --title "IBVSController"	--command "bash -c \"
						roslaunch parrotController parrot_IBVSController.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK}; 
						exec bash\""  \
	--tab --title "EKF"	--command "bash -c \"
	roslaunch parrotStateEstimator parrotStateEstimator.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK};
						exec bash\""	\
	--tab --title "IBVSCntInterf"	--command "bash -c \"
						roslaunch parrotBrainInterface parrotIBVSBrainInterface.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK}; 
						exec bash\""	&
