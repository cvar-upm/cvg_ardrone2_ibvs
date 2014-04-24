cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#http://answers.ros.org/question/11294/a-problem-of-linking-external-libraries/
#gcc -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__ -Wall -pipe -lnative -L/usr/xenomai/lib -lxenomai -lpthread -lrtdk hellworld.c -o helloworld
#link_directories(/usr/xenomai/lib)
#rosbuild_add_executable(listener src/listener.cpp)
#include_directories(/usr/xenomai/include)
#add_definitions(-D_GNU_SOURCE -D_REENTRANT -D__XENO__)
#rosbuild_add_compile_flags(listener -Wall -pipe)
#target_link_libraries(listener native xenomai pthread rtdk)






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





#Parrot controller compile
include_directories(${DRONESTATEESTIMATOR_INCLUDE_DIR})
rosbuild_add_library(droneStateEstimator ${DRONESTATEESTIMATOR_SOURCE_FILES} ${DRONESTATEESTIMATOR_HEADER_FILES})

set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})





