cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneStateEstimator)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
# add_definitions(-std=c++11)
add_definitions(-std=c++0x)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries






set(DRONESTATEESTIMATOR_SOURCE_DIR
	src/sources)
	
set(DRONESTATEESTIMATOR_INCLUDE_DIR
	src/include
	src/include/Other
	src/include/LMrT_poseConverter
	src/include/stateObserver
	src/include/stateObserver/EKF_lib
	src/include/stateObserver/models
	)

set(DRONESTATEESTIMATOR_SOURCE_FILES

	#General
	src/sources/droneStateEstimator.cpp
	
	#State Observer
	src/sources/stateObserver/EKF_lib/matrixLib.cpp 
	src/sources/stateObserver/EKF_lib/model.cpp
	src/sources/stateObserver/EKF_lib/extendedKalmanFilter.cpp
	src/sources/stateObserver/models/EKF_model_LinkQuad.cpp
	src/sources/stateObserver/models/EKF_model_Parrot.cpp
	src/sources/stateObserver/models/EKF_model_Pelican.cpp
	
	
	src/sources/LMrT_poseConverter/ekf_lmrt_pose_publisher.cpp

	
	#Other
	src/sources/Other/jesus_library.cpp
	
	
	)
	
set(DRONESTATEESTIMATOR_HEADER_FILES

	#General
	src/include/droneStateEstimator.h
	
	#Other
	src/include/Other/jesus_library.h
	
	
	src/include/LMrT_poseConverter/ekf_lmrt_pose_publisher.h
	
	
	#State Observer
	src/include/stateObserver/EKF_lib/extendedKalmanFilter.h
	src/include/stateObserver/EKF_lib/matrixLib.h
	src/include/stateObserver/EKF_lib/model.h
	src/include/stateObserver/models/EKF_config_Parrot.h
	src/include/stateObserver/models/EKF_model_Parrot.h
	src/include/stateObserver/models/EKF_config_Pelican.h
	src/include/stateObserver/models/EKF_model_Pelican.h
	src/include/stateObserver/models/EKF_config_LinkQuad.h
	src/include/stateObserver/models/EKF_model_LinkQuad.h
	
	)


find_package(catkin REQUIRED
	COMPONENTS message_generation roscpp std_msgs droneMsgs referenceFrames droneModule pugixml)


#OpenCV
find_package(OpenCV REQUIRED)



add_service_files(
    DIRECTORY
        srv
    FILES
	setInitDroneYaw_srv_type.srv
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
        INCLUDE_DIRS ${DRONESTATEESTIMATOR_INCLUDE_DIR}
        LIBRARIES droneStateEstimator
        CATKIN_DEPENDS message_runtime roscpp std_msgs droneMsgs referenceFrames droneModule pugixml
	DEPENDS OpenCV
  )



include_directories(${DRONESTATEESTIMATOR_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})





add_library(droneStateEstimator ${DRONESTATEESTIMATOR_SOURCE_FILES} ${DRONESTATEESTIMATOR_HEADER_FILES})
add_dependencies(droneStateEstimator ${catkin_EXPORTED_TARGETS})
add_dependencies(droneStateEstimator droneStateEstimator_gencpp)
target_link_libraries(droneStateEstimator ${OpenCV_LIBS})
target_link_libraries(droneStateEstimator ${catkin_LIBRARIES})






