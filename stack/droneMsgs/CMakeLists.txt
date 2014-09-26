cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneMsgs)
project(${PROJECT_NAME})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




set(DRONEMSGS_SOURCE_DIR
	src/sources)
	
set(DRONEMSGS_INCLUDE_DIR
	src/include
	)

set(DRONEMSGS_SOURCE_FILES

	)
	
set(DRONEMSGS_HEADER_FILES

        src/include/communication_definition.h
	)



find_package(catkin REQUIRED
                COMPONENTS message_generation std_msgs sensor_msgs geometry_msgs)



add_message_files(
    DIRECTORY
        msg
    FILES
        droneGoTask.msg                        
	droneSpeeds.msg
	droneHLCommandAck.msg                  
	droneTrajectoryRefCommand.msg
	droneHLCommand.msg                     
	droneYawRefCommand.msg
	droneInfo.msg                          
	imageFeaturesIBVS.msg
	droneMissionInfo.msg                   
	Landmark3D.msg
	droneNavCommand.msg                    
	landmarkVector.msg
	droneNavData.msg                       
	Observation3D.msg
	dronePose.msg                          
	obstaclesTwoDim.msg
	dronePositionRefCommand.msg            
	obstacleTwoDimPole.msg
	dronePositionTrajectoryRefCommand.msg  
	obstacleTwoDimWall.msg
	droneRefCommand.msg                    
	obsVector.msg
	droneSensorData.msg                    
	societyPose.msg



)



generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)




catkin_package(
        INCLUDE_DIRS ${DRONEMSGS_INCLUDE_DIR}
        CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs
	)



include_directories(${DRONEMSGS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
