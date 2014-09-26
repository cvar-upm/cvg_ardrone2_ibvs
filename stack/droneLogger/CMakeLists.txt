cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneLogger)
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




set(DRONELOGGER_INCLUDE_DIR src/include )

set(DRONELOGGER_HEADER_FILES
        src/include/DroneLogger.h   # IBVS clase for droneControllers
        )
set(DRONELOGGER_SOURCE_FILES
        src/sources/DroneLogger.cpp     # IBVS clase for droneControllers
        )



find_package(catkin REQUIRED
	COMPONENTS message_generation roscpp std_msgs atlante droneModule)

find_package(Boost REQUIRED COMPONENTS system filesystem)



add_service_files(
    DIRECTORY
        srv
    FILES
	getCLogPathInitTStamp.srv
	logThisString.srv
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
        INCLUDE_DIRS ${DRONELOGGER_INCLUDE_DIR}
        LIBRARIES droneLogger
        CATKIN_DEPENDS message_runtime roscpp std_msgs atlante droneModule
	DEPENDS Boost
  )



include_directories(${DRONELOGGER_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})




add_library(droneLogger ${DRONELOGGER_SOURCE_FILES} ${DRONELOGGER_HEADER_FILES})
add_dependencies(droneLogger ${catkin_EXPORTED_TARGETS})
add_dependencies(droneLogger droneLogger_gencpp)
target_link_libraries(droneLogger ${Boost_LIBRARIES})      
target_link_libraries(droneLogger ${catkin_LIBRARIES})



add_executable( main_tests src/sources/main_tests.cpp )
add_dependencies(main_tests ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_tests droneLogger)
target_link_libraries(main_tests ${Boost_LIBRARIES})
target_link_libraries(main_tests ${catkin_LIBRARIES})

