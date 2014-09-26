cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME parrotStateEstimator)
project(${PROJECT_NAME})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
# add_definitions(-std=c++11)
add_definitions(-std=c++0x)



set(PARROTSTATEESTIMATOR_SOURCE_DIR
	src/sources)
	
set(PARROTSTATEESTIMATOR_INCLUDE_DIR
	src/include)

set(PARROTSTATEESTIMATOR_SOURCE_FILES
	#General
	src/sources/parrotStateEstimator_Node.cpp
	)
	
set(PARROTSTATEESTIMATOR_HEADER_FILES
	)



find_package(catkin REQUIRED
	COMPONENTS parrotDriver droneStateEstimator droneLogger)



catkin_package(
        CATKIN_DEPENDS parrotDriver droneStateEstimator droneLogger
  )


include_directories(${catkin_INCLUDE_DIRS})




add_executable(parrotStateEstimator ${PARROTSTATEESTIMATOR_SOURCE_FILES} ${PARROTSTATEESTIMATOR_HEADER_FILES})
add_dependencies(parrotStateEstimator ${catkin_EXPORTED_TARGETS})    
target_link_libraries(parrotStateEstimator ${catkin_LIBRARIES})

add_executable(parrotStateEstimatorWLog src/sources/parrotStateEstimator_Node_WLog.cpp)
add_dependencies(parrotStateEstimatorWLog ${catkin_EXPORTED_TARGETS})    
target_link_libraries(parrotStateEstimatorWLog ${catkin_LIBRARIES})


