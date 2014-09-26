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






set(REFERENCEFRAMES_SOURCE_DIR
	src/sources)
	
set(REFERENCEFRAMES_INCLUDE_DIR
	src/include
	)

set(REFERENCEFRAMES_SOURCE_FILES

	#General
	src/sources/referenceFrames.cpp
	)
	
set(REFERENCEFRAMES_HEADER_FILES

	#General
	src/include/referenceFrames.h

	)


#openCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#### atlante lib #######



#add include directories to the serch path!!
#rosbuild_add_boost_directories()


#external libraries compile
#Atlante
#link_directories(${ATLANTE_LIB_DIR})
#include_directories(${ATLANTE_INCLUDE_DIR})



#Parrot controller compile
include_directories(${REFERENCEFRAMES_INCLUDE_DIR})
rosbuild_add_library(referenceFrames ${REFERENCEFRAMES_SOURCE_FILES} ${REFERENCEFRAMES_HEADER_FILES})
target_link_libraries(referenceFrames ${OpenCV_LIBS})



#external libraries link
#Atlante
#target_link_libraries(droneController ${ATLANTE_LIB_BIN})

set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})


