cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME parrotController)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
# add_definitions(-std=c++11)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries



set(PARROTIBVSCONTROLLER_SOURCE_FILES   
	src/sources/IBVScontrollerNodeMain.cpp )



find_package(catkin REQUIRED
	COMPONENTS atlante parrotDriver droneController droneStateEstimator droneOpenTLDInterface droneLogger)


#OpenCV
find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS atlante parrotDriver droneController droneStateEstimator droneOpenTLDInterface droneLogger
        DEPENDS OpenCV
  )


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})





add_executable(parrotIBVSController ${PARROTIBVSCONTROLLER_SOURCE_FILES})
add_dependencies(parrotIBVSController ${catkin_EXPORTED_TARGETS})    
target_link_libraries(parrotIBVSController ${OpenCV_LIBS})
target_link_libraries(parrotIBVSController ${catkin_LIBRARIES})

