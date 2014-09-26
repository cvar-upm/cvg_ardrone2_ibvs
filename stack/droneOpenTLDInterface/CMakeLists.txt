cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneOpenTLDInterface)
project(${PROJECT_NAME})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries



set(DRONEOPENTLDINTERFACE_INCLUDE_DIR
        src/include
        )

set(DRONEOPENTLDINTERFACE_SOURCE_FILES
        src/include/droneopentldinterface.h
        src/sources/droneopentldinterface.cpp
        )



find_package(catkin REQUIRED
	COMPONENTS roscpp std_msgs tld_msgs atlante droneModule)




catkin_package(
        INCLUDE_DIRS ${DRONEOPENTLDINTERFACE_INCLUDE_DIR}
        LIBRARIES droneOpenTLDInterface
        CATKIN_DEPENDS roscpp std_msgs tld_msgs atlante droneModule
  )



include_directories(${DRONEOPENTLDINTERFACE_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})



# DroneOpenTLDInterface library compile

add_library(droneOpenTLDInterface ${DRONEOPENTLDINTERFACE_SOURCE_FILES})
add_dependencies(droneOpenTLDInterface ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneOpenTLDInterface ${catkin_LIBRARIES})
