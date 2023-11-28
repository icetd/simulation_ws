# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "manager_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-Imanager_msgs:/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(manager_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg" NAME_WE)
add_custom_target(_manager_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manager_msgs" "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg" "manager_msgs/Type:manager_msgs/Quaternion:std_msgs/Header:manager_msgs/Point:manager_msgs/Pose"
)

get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg" NAME_WE)
add_custom_target(_manager_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manager_msgs" "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg" ""
)

get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg" NAME_WE)
add_custom_target(_manager_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manager_msgs" "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg" "manager_msgs/Point:manager_msgs/Quaternion"
)

get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg" NAME_WE)
add_custom_target(_manager_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manager_msgs" "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg" ""
)

get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg" NAME_WE)
add_custom_target(_manager_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manager_msgs" "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg" NAME_WE)
add_custom_target(_manager_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manager_msgs" "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs
)
_generate_msg_cpp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs
)
_generate_msg_cpp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs
)
_generate_msg_cpp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs
)
_generate_msg_cpp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs
)
_generate_msg_cpp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(manager_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(manager_msgs_generate_messages manager_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_cpp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_cpp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_cpp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_cpp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_cpp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_cpp _manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manager_msgs_gencpp)
add_dependencies(manager_msgs_gencpp manager_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manager_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs
)
_generate_msg_eus(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs
)
_generate_msg_eus(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs
)
_generate_msg_eus(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs
)
_generate_msg_eus(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs
)
_generate_msg_eus(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(manager_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(manager_msgs_generate_messages manager_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_eus _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_eus _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_eus _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_eus _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_eus _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_eus _manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manager_msgs_geneus)
add_dependencies(manager_msgs_geneus manager_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manager_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs
)
_generate_msg_lisp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs
)
_generate_msg_lisp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs
)
_generate_msg_lisp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs
)
_generate_msg_lisp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs
)
_generate_msg_lisp(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(manager_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(manager_msgs_generate_messages manager_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_lisp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_lisp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_lisp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_lisp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_lisp _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_lisp _manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manager_msgs_genlisp)
add_dependencies(manager_msgs_genlisp manager_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manager_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs
)
_generate_msg_nodejs(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs
)
_generate_msg_nodejs(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs
)
_generate_msg_nodejs(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs
)
_generate_msg_nodejs(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs
)
_generate_msg_nodejs(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(manager_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(manager_msgs_generate_messages manager_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_nodejs _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_nodejs _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_nodejs _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_nodejs _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_nodejs _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_nodejs _manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manager_msgs_gennodejs)
add_dependencies(manager_msgs_gennodejs manager_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manager_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs
)
_generate_msg_py(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs
)
_generate_msg_py(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg;/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs
)
_generate_msg_py(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs
)
_generate_msg_py(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs
)
_generate_msg_py(manager_msgs
  "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(manager_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(manager_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(manager_msgs_generate_messages manager_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Plan.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_py _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Point.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_py _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_py _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Quaternion.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_py _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Status.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_py _manager_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/share/Dev/simulation_ws/src/manager_msgs/msg/Type.msg" NAME_WE)
add_dependencies(manager_msgs_generate_messages_py _manager_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manager_msgs_genpy)
add_dependencies(manager_msgs_genpy manager_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manager_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manager_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(manager_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/manager_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(manager_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/manager_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(manager_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/manager_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(manager_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manager_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(manager_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
