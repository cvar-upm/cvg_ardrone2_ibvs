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






set(DRONEMSGS_SOURCE_DIR
	src/sources)
	
set(DRONEMSGS_INCLUDE_DIR
	src/include
	)

set(DRONEMSGS_SOURCE_FILES

	#General
	#src/sources/droneModule.cpp
	)
	
set(DRONEMSGS_HEADER_FILES

	#General
	#src/include/droneModule.h
        src/include/communication_definition.h
	)



#### atlante lib #######



#add include directories to the serch path!!
#rosbuild_add_boost_directories()


#external libraries compile
#Atlante
#link_directories(${ATLANTE_LIB_DIR})
#include_directories(${ATLANTE_INCLUDE_DIR})



#Parrot controller compile
include_directories(${DRONEMSGS_INCLUDE_DIR})
# rosbuild_add_library(droneMsgs ${DRONEMSGS_SOURCE_FILES} ${DRONEMSGS_HEADER_FILES})

set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})


#external libraries link
#Atlante
#target_link_libraries(droneController ${ATLANTE_LIB_BIN})




