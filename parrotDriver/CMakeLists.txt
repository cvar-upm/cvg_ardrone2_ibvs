cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

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



#external libraries
#ncurses
#link_directories(/usr/lib/x86_64-linux-gnu)
#include_directories(/usr/include)


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



#include_directories(src/include/)
#rosbuild_add_library(parrotDriver src/Drone.cpp)

include_directories(${PARROTDRIVER_INCLUDE_DIR})
rosbuild_add_library(parrotDriver ${PARROTDRIVER_SOURCE_FILES} ${PARROTDRIVER_HEADER_FILES})


#rosbuild_add_executable(parrotInterface src/interface.cpp src/Drone.cpp src/Drone.h)
#rosbuild_add_executable(parrotInterface src/interface.cpp)

#rosbuild_add_executable(parrotInterfaceCommand src/interfaceCommand.cpp)


#external libraries
#ncurses
#target_link_libraries(parrotInterface ncurses)

set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})

