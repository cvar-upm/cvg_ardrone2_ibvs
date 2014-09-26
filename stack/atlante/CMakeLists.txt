cmake_minimum_required(VERSION 2.4.6)

set(PROJECT_NAME atlante)
project(${PROJECT_NAME})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries



# poshlib
set (POSHLIB_PATH "${PROJECT_SOURCE_DIR}/3rdparty/poshlib" )

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



find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include ${POSHLIB_PATH}
    LIBRARIES atlante
  )



# include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${POSHLIB_PATH})

include_directories(${catkin_INCLUDE_DIRS})





add_library(atlante ${ATLANTE_FRAMEWORK_SRCs} ${ATLANTE_FRAMEWORK_INCLUDEs})
add_dependencies(atlante ${catkin_EXPORTED_TARGETS})
target_link_libraries(atlante ${catkin_LIBRARIES})




