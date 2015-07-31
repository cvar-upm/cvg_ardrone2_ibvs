#!/bin/bash
echo "Configuring stack submodules .."


CONFIG_SUBMODULES_FILE=$1


#Reading configuration of which submodules include
. $CONFIG_SUBMODULES_FILE



#Adquire bitbucket info
echo "Acquiring bitbucket user info"
echo -n " -Bitbucket username: "
read bitbucketUsername

echo -n " -Bitbucket password: "
read -s bitbucketPassword
echo ""



#Loop for git submodule init
echo "Adding submodules"


#ardrone_autonomy
if [[ ${ardrone_autonomy}  && $ardrone_autonomy = true ]]
	then
		MODULE_PATH=stack/droneDriver/DriveParrotARDrone/ardrone_autonomy
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitPublRepo.sh $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#driverParrotARDroneROSModule
if [[ ${driverParrotARDroneROSModule}  && $driverParrotARDroneROSModule = true ]]
	then
		MODULE_PATH=stack/droneDriver/DriveParrotARDrone/driverParrotARDroneROSModule
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi


#droneIBVSController
if [[ ${droneIBVSController}  && $droneIBVSController = true ]]
	then
		MODULE_PATH=stack/droneIBVSControl/droneIBVSController
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneIBVSControllerROSModule
if [[ ${droneIBVSControllerROSModule}  && $droneIBVSControllerROSModule = true ]]
	then
		MODULE_PATH=stack/droneIBVSControl/droneIBVSControllerROSModule
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneTrackerEyeROSModule
if [[ ${droneTrackerEyeROSModule}  && $droneTrackerEyeROSModule = true ]]
	then
		MODULE_PATH=stack/droneIBVSControl/droneTrackerEyeROSModule
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi


#droneLoggerROSModule
if [[ ${droneLoggerROSModule}  && $droneLoggerROSModule = true ]]
	then
		MODULE_PATH=stack/droneLogging/droneLoggerROSModule
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi


#lib_cvglogger
if [[ ${lib_cvglogger}  && $lib_cvglogger = true ]]
	then
		MODULE_PATH=stack/droneLogging/lib_cvglogger
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_cvgloggerROS
if [[ ${lib_cvgloggerROS}  && $lib_cvgloggerROS = true ]]
	then
		MODULE_PATH=stack/droneLogging/lib_cvgloggerROS
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi


#droneOpenTLDTranslatorROS
if [[ ${droneOpenTLDTranslatorROS}  && $droneOpenTLDTranslatorROS = true ]]
	then
		MODULE_PATH=stack/dronePerception/openTLD/droneOpenTLDTranslatorROS
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#ros_opentld
if [[ ${ros_opentld}  && $ros_opentld = true ]]
	then
		MODULE_PATH=stack/dronePerception/openTLD/ros_opentld
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneEKFStateEstimator
if [[ ${droneEKFStateEstimator}  && $droneEKFStateEstimator = true ]]
	then
		MODULE_PATH=stack/droneSelfLocalization/droneOdometryPoseEstimator/droneEKFStateEstimator
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneEKFStateEstimatorROSModule
if [[ ${droneEKFStateEstimatorROSModule}  && $droneEKFStateEstimatorROSModule = true ]]
	then
		MODULE_PATH=stack/droneSelfLocalization/droneOdometryPoseEstimator/droneEKFStateEstimatorROSModule
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneModuleInterfaceROS
if [[ ${droneModuleInterfaceROS}  && $droneModuleInterfaceROS = true ]]
	then
		MODULE_PATH=stack/droneStackBasics/droneModuleInterfaceROS
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneModuleROS
if [[ ${droneModuleROS}  && $droneModuleROS = true ]]
	then
		MODULE_PATH=stack/droneStackBasics/droneModuleROS
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneMsgsROS
if [[ ${droneMsgsROS}  && $droneMsgsROS = true ]]
	then
		MODULE_PATH=stack/droneStackBasics/droneMsgsROS
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneInterfaceROSModule
if [[ ${droneInterfaceROSModule}  && $droneInterfaceROSModule = true ]]
	then
		MODULE_PATH=stack/HMI/droneInterfaceROSModule
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_cvgekf
if [[ ${lib_cvgekf}  && $lib_cvgekf = true ]]
	then
		MODULE_PATH=stack/libraries/lib_cvgekf
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_cvgutils
if [[ ${lib_cvgutils}  && $lib_cvgutils = true ]]
	then
		MODULE_PATH=stack/libraries/lib_cvgutils
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_pugixml
if [[ ${lib_pugixml}  && $lib_pugixml = true ]]
	then
		MODULE_PATH=stack/libraries/lib_pugixml
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#referenceFrames
if [[ ${referenceFrames}  && $referenceFrames = true ]]
	then
		MODULE_PATH=stack/libraries/referenceFrames
		# git submodule deinit $MODULE_PATH > /dev/null
		./installation/gitSubmoduleUpdateInitBitbucketPrivRepo.sh $MODULE_PATH $bitbucketUsername $bitbucketPassword > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi
