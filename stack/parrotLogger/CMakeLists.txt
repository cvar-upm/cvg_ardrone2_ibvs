cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME parrotLogger)
project(${PROJECT_NAME})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




set(PARROTLOGGER_SOURCE_FILES src/sources/parrotLoggerNodeMain.cpp)
set(PARROTLOGGER_HEADER_FILES   )



find_package(catkin REQUIRED
	COMPONENTS atlante droneLogger)



catkin_package(
        CATKIN_DEPENDS atlante droneLogger
  )


include_directories(${catkin_INCLUDE_DIRS})


add_executable(parrotLogger ${PARROTLOGGER_SOURCE_FILES} ${PARROTLOGGER_HEADER_FILES})
add_dependencies(parrotLogger ${catkin_EXPORTED_TARGETS})    
target_link_libraries(parrotLogger ${catkin_LIBRARIES})

