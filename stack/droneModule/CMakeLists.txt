cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneModule)
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



set(DRONEMODULE_SOURCE_DIR
	src/sources)
	
set(DRONEMODULE_INCLUDE_DIR
	src/include
	)

set(DRONEMODULE_SOURCE_FILES

	#General
	src/sources/droneModule.cpp
	)
	
set(DRONEMODULE_HEADER_FILES

	#General
	src/include/droneModule.h

	)



find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS ${DRONEMODULE_INCLUDE_DIR}
    LIBRARIES droneModule
  )



# include directories
include_directories(${DRONEMODULE_INCLUDE_DIR})

include_directories(${catkin_INCLUDE_DIRS})




add_library(droneModule ${DRONEMODULE_SOURCE_FILES} ${DRONEMODULE_HEADER_FILES})
add_dependencies(droneModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneModule ${catkin_LIBRARIES})

