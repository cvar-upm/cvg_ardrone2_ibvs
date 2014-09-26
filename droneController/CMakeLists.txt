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
rosbuild_gensrv()

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

set(PARROTCONTROLLER_SOURCE_DIR
	src/sources)
	
set(PARROTCONTROLLER_INCLUDE_DIR
        src/include
        src/include/controller/other
        src/include/Other
	)

set(DRONEIBVSCONTROLLER_HEADER_FILES
        src/include/droneIBVSController.h   # IBVS clase for droneControllers
        src/include/Other/Timer.h
        src/include/controller/other/LowPassFilter.h
        src/include/controller/other/PID.h
        src/include/Other/jesus_library.h
        src/include/stateObserver/EKF_lib/matrixLib.h
        )
set(DRONEIBVSCONTROLLER_SOURCE_FILES
        src/sources/droneIBVSController.cpp     # IBVS clase for droneControllers
        src/sources/controller/Other/LowPassFilter.cpp
        src/sources/controller/Other/PID.cpp
        src/sources/Other/jesus_library.cpp
        src/sources/stateObserver/EKF_lib/matrixLib.cpp
        )

#### atlante lib #######
#set(ATLANTE_INCLUDE_DIR
#	atlante
#	atlante/include
#	)
	
#set(ATLANTE_LIB_DIR
#	atlante/lib/gcc
#	)

#set(ATLANTE_LIB_BIN
#	${PROJECT_SOURCE_DIR}/atlante/lib/gcc/libatlante.a
#	)

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#add include directories to the serch path!!
rosbuild_add_boost_directories()


#external libraries compile
#Atlante
#link_directories(${ATLANTE_LIB_DIR})
#include_directories(${ATLANTE_INCLUDE_DIR})



#Parrot controller compile
include_directories(${PARROTCONTROLLER_INCLUDE_DIR})
rosbuild_add_library(droneController    ${DRONEIBVSCONTROLLER_SOURCE_FILES}
                                        ${DRONEIBVSCONTROLLER_HEADER_FILES}
                                        )
target_link_libraries(droneController ${OpenCV_LIBS})



#external libraries link
#Atlante
#target_link_libraries(droneController ${ATLANTE_LIB_BIN})




