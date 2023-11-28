cmake_minimum_required(VERSION 3.0.2)
project(manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)
set(CMKAE_C_FLAGS "${CMKAE_C_FLAGS} -O2")
set(CMKAE_CXX_FLAGS "${CMKAE_CXX_FLAGS} -O2")

set(SOURCE_DIR  "${CMAKE_CURRENT_LIST_DIR}/src")
set(INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/src")

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
    "${SOURCE_DIR}/*.cpp"
)

file (GLOB_RECURSE INC_FILES
    "${INCLUDE_DIR}/*.h"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)


add_executable(manager ${SRC_FILES} ${INC_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${SOURCE_DIR})

target_link_libraries(manager
  ${catkin_LIBRARIES}
)

