#!/bin/bash

# Load ROS_WORKSPACE, etc
. ${IBVS_STACK}/setup.sh

# Load some local variables
NUMID_DRONE=1
NETWORK_ROSCORE=localhost
export ROS_MASTER_URI=http://localhost:11311
OPEN_ROSCORE=1

gnome-terminal	 		\
	--tab --title "roscore" 	--command "bash -c \"
						roscore; 
						exec bash\""  \
	--tab --title "ardrone_driver"	--command "bash -c \"
						roslaunch ${IBVS_STACK}/launchers/ardrone_launch/ardrone_indoors.launch --wait drone_id_namespace:=drone$NUMID_DRONE;
						exec bash\""  \
         --tab --title "ardrone_driver_ros_module"   --command "bash -c \"
						roslaunch driverParrotARDroneROSModule driverParrotARDroneROSModule.launch --wait drone_id_namespace:=drone$NUMID_DRONE;
						exec bash\""  \
	--tab --title "OpenTLD"		--command "bash -c \" 
						roslaunch ${IBVS_STACK}/launchers/ardrone_launch/launch_files/opentld_for_IBVSController.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK}; 
						exec bash\""  \
	--tab --title "IBVSController"	--command "bash -c \"
						roslaunch droneIBVSControllerROSModule DroneIBVSControllerROSModule.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK}; 
						exec bash\""  \
	--tab --title "EKF"	--command "bash -c \"
	roslaunch droneEKFStateEstimatorROSModule droneEKFStateEstimatorROSModule.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK};
						exec bash\""	\
	--tab --title "IBVSCntInterf"	--command "bash -c \"
						roslaunch droneInterfaceROSModule droneInterface_jp_ROSModule.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK}; 
						exec bash\""	\
	--tab --title "droneLogger"	--command "bash -c \"
						roslaunch ${IBVS_STACK}/launchers/ardrone_launch/launch_files/droneLoggerROSModule.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_id_int:=$NUMID_DRONE my_stack_directory:=${IBVS_STACK}; 
						exec bash\""   &
