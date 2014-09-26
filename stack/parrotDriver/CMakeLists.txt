cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME parrotDriver)
project(${PROJECT_NAME})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




set(PARROTDRIVER_SOURCE_DIR
	src)
	
set(PARROTDRIVER_INCLUDE_DIR
	src/include
	)

set(PARROTDRIVER_SOURCE_FILES
	src/source/Drone.cpp
	)
	
set(PARROTDRIVER_HEADER_FILES
	src/include/Drone.h
	)


find_package(catkin REQUIRED
	COMPONENTS ardrone_autonomy roscpp std_msgs sensor_msgs cv_bridge image_transport droneMsgs)

#OpenCV
find_package(OpenCV REQUIRED)


catkin_package(
        INCLUDE_DIRS ${PARROTDRIVER_INCLUDE_DIR}
        LIBRARIES parrotDriver
	DEPENDS OpenCV
        CATKIN_DEPENDS ardrone_autonomy roscpp std_msgs sensor_msgs cv_bridge image_transport droneMsgs
  )



include_directories(${PARROTDRIVER_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})







add_library(parrotDriver ${PARROTDRIVER_SOURCE_FILES} ${PARROTDRIVER_HEADER_FILES})
add_dependencies(parrotDriver ${catkin_EXPORTED_TARGETS})
target_link_libraries(parrotDriver ${OpenCV_LIBS})
target_link_libraries(parrotDriver ${catkin_LIBRARIES})




