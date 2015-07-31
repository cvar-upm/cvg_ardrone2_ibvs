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
		REPO_URL=https://github.com/AutonomyLab/ardrone_autonomy.git
		REPO_BRANCH=indigo-devel
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#driverParrotARDroneROSModule
if [[ ${driverParrotARDroneROSModule}  && $driverParrotARDroneROSModule = true ]]
	then
		MODULE_PATH=stack/droneDriver/DriveParrotARDrone/driverParrotARDroneROSModule
		REPO_URL=https://bitbucket.org/joselusl/driverparrotardronerosmodule.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneIBVSController
if [[ ${droneIBVSController}  && $droneIBVSController = true ]]
	then
		MODULE_PATH=stack/droneIBVSControl/droneIBVSController
		REPO_URL=https://bitbucket.org/hridaybavle/droneibvscontroller.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneIBVSControllerROSModule
if [[ ${droneIBVSControllerROSModule}  && $droneIBVSControllerROSModule = true ]]
	then
		MODULE_PATH=stack/droneIBVSControl/droneIBVSControllerROSModule
		REPO_URL=https://bitbucket.org/hridaybavle/droneibvscontrollerrosmodule.git
		REPO_BRANCH=IBVS
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneTrackerEyeROSModule
if [[ ${droneTrackerEyeROSModule}  && $droneTrackerEyeROSModule = true ]]
	then
		MODULE_PATH=stack/droneIBVSControl/droneTrackerEyeROSModule
		REPO_URL=https://bitbucket.org/hridaybavle/dronetrackereyerosmodule.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneLoggerROSModule
if [[ ${droneLoggerROSModule}  && $droneLoggerROSModule = true ]]
	then
		MODULE_PATH=stack/droneLogging/droneLoggerROSModule
		REPO_URL=https://bitbucket.org/jespestana/droneloggerrosmodule.git
		REPO_BRANCH=IBVS
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_cvglogger
if [[ ${lib_cvglogger}  && $lib_cvglogger = true ]]
	then
		MODULE_PATH=stack/droneLogging/lib_cvglogger
		REPO_URL=https://bitbucket.org/jespestana/lib_cvglogger.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_cvgloggerROS
if [[ ${lib_cvgloggerROS}  && $lib_cvgloggerROS = true ]]
	then
		MODULE_PATH=stack/droneLogging/lib_cvgloggerROS
		REPO_URL=https://bitbucket.org/jespestana/lib_cvgloggerros.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi


#droneOpenTLDTranslatorROS
if [[ ${droneOpenTLDTranslatorROS}  && $droneOpenTLDTranslatorROS = true ]]
	then
		MODULE_PATH=stack/dronePerception/openTLD/droneOpenTLDTranslatorROS
		REPO_URL=https://bitbucket.org/hridaybavle/droneopentldtranslatorros.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#ros_opentld
if [[ ${ros_opentld}  && $ros_opentld = true ]]
	then
		MODULE_PATH=stack/dronePerception/openTLD/ros_opentld
		REPO_URL=https://github.com/Ronan0912/ros_opentld.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneEKFStateEstimator
if [[ ${droneEKFStateEstimator}  && $droneEKFStateEstimator = true ]]
	then
		MODULE_PATH=stack/droneSelfLocalization/droneOdometryPoseEstimator/droneEKFStateEstimator
		REPO_URL=https://bitbucket.org/jespestana/droneekfstateestimator.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneEKFStateEstimatorROSModule
if [[ ${droneEKFStateEstimatorROSModule}  && $droneEKFStateEstimatorROSModule = true ]]
	then
		MODULE_PATH=stack/droneSelfLocalization/droneOdometryPoseEstimator/droneEKFStateEstimatorROSModule
		REPO_URL=https://bitbucket.org/jespestana/droneekfstateestimatorrosmodule.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneModuleInterfaceROS
if [[ ${droneModuleInterfaceROS}  && $droneModuleInterfaceROS = true ]]
	then
		MODULE_PATH=stack/droneStackBasics/droneModuleInterfaceROS
		REPO_URL=https://bitbucket.org/jespestana/dronemoduleinterfaceros.git
		REPO_BRANCH=IBVS
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneModuleROS
if [[ ${droneModuleROS}  && $droneModuleROS = true ]]
	then
		MODULE_PATH=stack/droneStackBasics/droneModuleROS
		REPO_URL=https://bitbucket.org/joselusl/dronemoduleros.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#droneMsgsROS
if [[ ${droneMsgsROS}  && $droneMsgsROS = true ]]
	then
		MODULE_PATH=stack/droneStackBasics/droneMsgsROS
		REPO_URL=https://bitbucket.org/joselusl/dronemsgsros.git
		REPO_BRANCH=IBVS
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi


#droneInterfaceROSModule
if [[ ${droneInterfaceROSModule}  && $droneInterfaceROSModule = true ]]
	then
		MODULE_PATH=stack/HMI/droneInterfaceROSModule
		REPO_URL=https://bitbucket.org/joselusl/droneinterfacerosmodule.git
		REPO_BRANCH=IBVS
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_cvgekf
if [[ ${lib_cvgekf}  && $lib_cvgekf = true ]]
	then
		MODULE_PATH=stack/libraries/lib_cvgekf
		REPO_URL=https://bitbucket.org/joselusl/lib_cvgekf.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_cvgutils
if [[ ${lib_cvgutils}  && $lib_cvgutils = true ]]
	then
		MODULE_PATH=stack/libraries/lib_cvgutils
		REPO_URL=https://bitbucket.org/jespestana/lib_cvgutils.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#lib_pugixml
if [[ ${lib_pugixml}  && $lib_pugixml = true ]]
	then
		MODULE_PATH=stack/libraries/lib_pugixml
		REPO_URL=https://bitbucket.org/joselusl/lib_pugixml.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi

#referenceFrames
if [[ ${referenceFrames}  && $referenceFrames = true ]]
	then
		MODULE_PATH=stack/libraries/referenceFrames
		REPO_URL=https://bitbucket.org/joselusl/referenceframes.git
		REPO_BRANCH=master
		./installation/gitSubmoduleAddPublRepo.sh $REPO_BRANCH $REPO_URL $MODULE_PATH > /dev/null
		echo " -Added package in: $MODULE_PATH"
fi
