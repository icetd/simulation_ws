cmake_minimum_required(VERSION 3.0.2)
project(manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)
set(CMKAE_C_FLAGS "${CMKAE_C_FLAGS} -O2")
set(CMKAE_CXX_FLAGS "${CMKAE_CXX_FLAGS} -O2")

set (EIGEN_INCLUDE  ${CMAKE_CURRENT_LIST_DIR}/thirdparty/eigen3)
set (aruco_DIR      ${CMAKE_CURRENT_LIST_DIR}/thirdparty/aruco)
set(SOURCE_DIR      ${CMAKE_CURRENT_LIST_DIR}/src)
set(INCLUDE_DIR     ${CMAKE_CURRENT_LIST_DIR}/src)

add_subdirectory(thirdparty/FFmpeg)
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib_msgs
  nav_msgs
  manager_msgs
)

catkin_package(
   CATKIN_DEPENDS 
   roscpp 
   std_msgs 
   geometry_msgs
   move_base_msgs
   actionlib_msgs
   nav_msgs
   manager_msgs
)

file(GLOB_RECURSE SRC_FILES
    "${SOURCE_DIR}/*.c*"
    "${SOURCE_DIR}/Core/*.c*"
    "${SOURCE_DIR}/Driver/*.c*"
    "${SOURCE_DIR}/Video/*.c*"
)

file (GLOB_RECURSE INC_FILES
    "${INCLUDE_DIR}/*.h"
    "${INCLUDE_DIR}/Core/*.h"
    "${INCLUDE_DIR}/Driver/*.h"
    "${INCLUDE_DIR}/Video/*.h"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  ${INCLUDE_DIR}/Core
  ${INCLUDE_DIR}/Driver
  ${INCLUDE_DIR}/Video
  ${EIGEN_INCLUDE}
  ${OpenCV_Include_DIRS}
	${ARUCO_INCLUDE}
)


add_executable(manager ${SRC_FILES} ${INC_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${SOURCE_DIR})

target_link_libraries(manager
  aruco ${OpenCV_LIBS} avformat swscale avdevice avformat avcodec avutil
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


