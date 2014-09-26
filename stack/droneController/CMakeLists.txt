cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneController)
project(${PROJECT_NAME})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




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



find_package(catkin REQUIRED
	COMPONENTS message_generation roscpp std_msgs atlante droneModule droneMsgs droneOpenTLDInterface)

#OpenCV
find_package(OpenCV REQUIRED)



add_service_files(
    DIRECTORY
        srv
    FILES
	setControlMode.srv
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
        INCLUDE_DIRS ${PARROTCONTROLLER_INCLUDE_DIR}
        LIBRARIES droneController
        CATKIN_DEPENDS message_runtime roscpp std_msgs atlante droneModule droneMsgs droneOpenTLDInterface
	DEPENDS OpenCV
  )



include_directories(${PARROTCONTROLLER_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})




add_library(droneController    ${DRONEIBVSCONTROLLER_SOURCE_FILES}
                                        ${DRONEIBVSCONTROLLER_HEADER_FILES}
                                        )
add_dependencies(droneController ${catkin_EXPORTED_TARGETS})
add_dependencies(droneController droneController_gencpp)
target_link_libraries(droneController ${OpenCV_LIBS})
target_link_libraries(droneController ${catkin_LIBRARIES})



