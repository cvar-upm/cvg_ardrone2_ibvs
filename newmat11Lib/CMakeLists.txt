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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv() 
 
 

#NEWMAT11 LIB
set(NEWMAT11_SOURCE_DIR
	src) 
	
set(NEWMAT11_INCLUDE_DIR
	src)

set(NEWMAT11_SOURCE_FILES
	
	${NEWMAT11_SOURCE_DIR}/bandmat.cpp   
	${NEWMAT11_SOURCE_DIR}/newmat1.cpp   
	${NEWMAT11_SOURCE_DIR}/newmatnl.cpp  
	${NEWMAT11_SOURCE_DIR}/submat.cpp    
	${NEWMAT11_SOURCE_DIR}/tmt8.cpp  
	${NEWMAT11_SOURCE_DIR}/tmth.cpp
	${NEWMAT11_SOURCE_DIR}/cholesky.cpp  
	${NEWMAT11_SOURCE_DIR}/newmat2.cpp   
	${NEWMAT11_SOURCE_DIR}/newmatrm.cpp  
	${NEWMAT11_SOURCE_DIR}/svd.cpp       
	${NEWMAT11_SOURCE_DIR}/tmt9.cpp  
	${NEWMAT11_SOURCE_DIR}/tmti.cpp
	${NEWMAT11_SOURCE_DIR}/evalue.cpp    
	${NEWMAT11_SOURCE_DIR}/newmat3.cpp   
	#${NEWMAT11_SOURCE_DIR}/nl_ex.cpp     
	#${NEWMAT11_SOURCE_DIR}/test_exc.cpp  
	${NEWMAT11_SOURCE_DIR}/tmta.cpp  
	${NEWMAT11_SOURCE_DIR}/tmtj.cpp
	#${NEWMAT11_SOURCE_DIR}/example.cpp   
	${NEWMAT11_SOURCE_DIR}/newmat4.cpp   
	#${NEWMAT11_SOURCE_DIR}/nm_ex1.cpp    
	${NEWMAT11_SOURCE_DIR}/tmt1.cpp      
	${NEWMAT11_SOURCE_DIR}/tmtb.cpp  
	${NEWMAT11_SOURCE_DIR}/tmtk.cpp
	${NEWMAT11_SOURCE_DIR}/fft.cpp       
	${NEWMAT11_SOURCE_DIR}/newmat5.cpp   
	#${NEWMAT11_SOURCE_DIR}/nm_ex2.cpp    
	${NEWMAT11_SOURCE_DIR}/tmt2.cpp      
	${NEWMAT11_SOURCE_DIR}/tmtc.cpp  
	${NEWMAT11_SOURCE_DIR}/tmtl.cpp
	#${NEWMAT11_SOURCE_DIR}/garch.cpp     
	${NEWMAT11_SOURCE_DIR}/newmat6.cpp   
	#${NEWMAT11_SOURCE_DIR}/nm_ex3.cpp    
	${NEWMAT11_SOURCE_DIR}/tmt3.cpp      
	${NEWMAT11_SOURCE_DIR}/tmt.cpp   
	${NEWMAT11_SOURCE_DIR}/tmtm.cpp
	${NEWMAT11_SOURCE_DIR}/hholder.cpp   
	${NEWMAT11_SOURCE_DIR}/newmat7.cpp   
	${NEWMAT11_SOURCE_DIR}/nm_misc.cpp   
	${NEWMAT11_SOURCE_DIR}/tmt4.cpp      
	${NEWMAT11_SOURCE_DIR}/tmtd.cpp
	${NEWMAT11_SOURCE_DIR}/jacobi.cpp    
	${NEWMAT11_SOURCE_DIR}/newmat8.cpp   
	#${NEWMAT11_SOURCE_DIR}/sl_ex.cpp     
	${NEWMAT11_SOURCE_DIR}/tmt5.cpp      
	${NEWMAT11_SOURCE_DIR}/tmte.cpp
	${NEWMAT11_SOURCE_DIR}/myexcept.cpp  
	${NEWMAT11_SOURCE_DIR}/newmat9.cpp   
	${NEWMAT11_SOURCE_DIR}/solution.cpp  
	${NEWMAT11_SOURCE_DIR}/tmt6.cpp      
	${NEWMAT11_SOURCE_DIR}/tmtf.cpp
	${NEWMAT11_SOURCE_DIR}/newfft.cpp    
	${NEWMAT11_SOURCE_DIR}/newmatex.cpp  
	${NEWMAT11_SOURCE_DIR}/sort.cpp      
	${NEWMAT11_SOURCE_DIR}/tmt7.cpp      
	${NEWMAT11_SOURCE_DIR}/tmtg.cpp
)
 
set(NEWMAT11_HEADER_FILES
	
	${NEWMAT11_INCLUDE_DIR}/controlw.h  
	${NEWMAT11_INCLUDE_DIR}/myexcept.h  
	${NEWMAT11_INCLUDE_DIR}/newmat.h    
	${NEWMAT11_INCLUDE_DIR}/newmatnl.h  
	${NEWMAT11_INCLUDE_DIR}/newmatrm.h  
	${NEWMAT11_INCLUDE_DIR}/solution.h
	${NEWMAT11_INCLUDE_DIR}/include.h   
	${NEWMAT11_INCLUDE_DIR}/newmatap.h  
	${NEWMAT11_INCLUDE_DIR}/newmatio.h  
	${NEWMAT11_INCLUDE_DIR}/newmatrc.h  
	${NEWMAT11_INCLUDE_DIR}/precisio.h  
	${NEWMAT11_INCLUDE_DIR}/tmt.h
)

 


 






include_directories(${NEWMAT11_INCLUDE_DIR})

 
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread) 
rosbuild_add_library(newmat11Lib ${NEWMAT11_SOURCE_FILES} ${NEWMAT11_HEADER_FILES})


#rosbuild_add_executable(ArUcoBoarder src/aruco_test_board.cpp ${ARUCO_SOURCE_FILES} ${ARUCO_HEADER_FILES})
#rosbuild_add_executable(ArUcoMarker src/aruco_simple.cpp ${ARUCO_SOURCE_FILES} ${ARUCO_HEADER_FILES})
#target_link_libraries(aruco ${PROJECT_NAME})

set(OTHER_FILES
        manifest.xml
)
add_custom_target(other_files SOURCES ${OTHER_FILES})
