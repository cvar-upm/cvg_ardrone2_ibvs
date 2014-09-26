cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneBrain)
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




set(DRONEBRAIN_SOURCE_DIR
	src/sources)
	
set(DRONEBRAIN_INCLUDE_DIR
	src/include
	)

set(DRONEBRAIN_MORE_SOURCE_FILES
                                    src/sources/droneIBVSBrain.cpp
                                    )

set(DRONEBRAIN_MORE_HEADER_FILES
                                    src/include/droneIBVSBrain.h
                                    )



find_package(catkin REQUIRED
	COMPONENTS droneController droneStateEstimator droneOpenTLDInterface ardrone_autonomy)



catkin_package(
        INCLUDE_DIRS ${DRONEBRAIN_INCLUDE_DIR}
        LIBRARIES droneBrain
        CATKIN_DEPENDS droneController droneStateEstimator droneOpenTLDInterface ardrone_autonomy
  )



include_directories(${DRONEBRAIN_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})





add_library(droneBrain
                                ${DRONEBRAIN_MORE_SOURCE_FILES}
                                ${DRONEBRAIN_MORE_HEADER_FILES}
                                )
add_dependencies(droneBrain ${catkin_EXPORTED_TARGETS})    
target_link_libraries(droneBrain ${catkin_LIBRARIES})





