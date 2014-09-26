cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME parrotBrainInterface)
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



find_package(catkin REQUIRED
	COMPONENTS parrotDriver droneBrain droneLogger)


catkin_package(
        CATKIN_DEPENDS parrotDriver droneBrain droneLogger
  )



#ncurses
link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include)

include_directories(${catkin_INCLUDE_DIRS})




add_executable(parrotBrainInterface_IBVS src/sources/IBVS_interface.cpp)
add_dependencies(parrotBrainInterface_IBVS ${catkin_EXPORTED_TARGETS})    
target_link_libraries(parrotBrainInterface_IBVS ncurses)
target_link_libraries(parrotBrainInterface_IBVS ${catkin_LIBRARIES})

