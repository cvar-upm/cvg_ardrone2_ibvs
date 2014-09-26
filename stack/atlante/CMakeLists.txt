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
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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

# The version number.
# set (IMG_ACQ_2CAMSTH_VERSION_MAJOR 1)
# set (IMG_ACQ_2CAMSTH_VERSION_MINOR 0)

# Create Eclipse project file
# message ("CMAKE_GENERATOR: " ${CMAKE_GENERATOR})
# set (CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT TRUE)
# set (CMAKE_GENERATOR "Eclipse CDT4 - Unix Makefiles")
# message ("CMAKE_GENERATOR: " ${CMAKE_GENERATOR})
# message ("CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT: " ${CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT} )

# message ("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR} )
# set (PROJECT_SOURCE_DIR "${PROJECT_SOURCE_DIR}/sources" )
set (PROJECT_BINARY_DIR "${PROJECT_SOURCE_DIR}/bin" )
# message ("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR} )

# include directories
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}")

# poshlib
set (POSHLIB_PATH "${PROJECT_SOURCE_DIR}/3rdparty/poshlib" )
include_directories("${POSHLIB_PATH}")

# set(CMAKE_SOURCE_DIR  "${SOURCE_DIR}/src/" CACHE)
set ( ATLANTE_FRAMEWORK_SRCs
        sources/cvgException.cpp
        sources/cvgMutex.cpp
        sources/cvgString.cpp
        sources/cvgThread.cpp
        sources/cvgTimer.cpp
        sources/HTMatrix4.cpp
        sources/Matrix3.cpp
        sources/Matrix4.cpp
        sources/RotMatrix3.cpp
        sources/Vector3.cpp
        sources/Vector4.cpp
        ${POSHLIB_PATH}/posh.c )

set ( ATLANTE_FRAMEWORK_INCLUDEs
        include/cvgCondition.h
        include/cvgException.h
        include/cvgMutex.h
        include/cvgSemaphore.h
        include/cvgString.h
        include/cvgThread.h
        include/cvgTimer.h
        include/cvg_types.h
        include/cvgVector.h
        include/HTMatrix4.h
        include/Matrix3.h
        include/Matrix4.h
        include/RotMatrix3.h
        include/Vector3.h
        include/Vector4.h )

add_library(atlante ${ATLANTE_FRAMEWORK_SRCs} ${ATLANTE_FRAMEWORK_INCLUDEs})
# target_link_libraries (imacq_2camSTH atlante)

set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})
