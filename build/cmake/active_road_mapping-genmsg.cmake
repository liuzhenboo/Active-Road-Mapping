# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "active_road_mapping: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iactive_road_mapping:/home/lzb/ros_ws/src/Active-Road-Mapping/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(active_road_mapping_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_custom_target(_active_road_mapping_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "active_road_mapping" "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(active_road_mapping
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_road_mapping
)

### Generating Services

### Generating Module File
_generate_module_cpp(active_road_mapping
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_road_mapping
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(active_road_mapping_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(active_road_mapping_generate_messages active_road_mapping_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(active_road_mapping_generate_messages_cpp _active_road_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_road_mapping_gencpp)
add_dependencies(active_road_mapping_gencpp active_road_mapping_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_road_mapping_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(active_road_mapping
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_road_mapping
)

### Generating Services

### Generating Module File
_generate_module_eus(active_road_mapping
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_road_mapping
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(active_road_mapping_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(active_road_mapping_generate_messages active_road_mapping_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(active_road_mapping_generate_messages_eus _active_road_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_road_mapping_geneus)
add_dependencies(active_road_mapping_geneus active_road_mapping_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_road_mapping_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(active_road_mapping
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_road_mapping
)

### Generating Services

### Generating Module File
_generate_module_lisp(active_road_mapping
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_road_mapping
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(active_road_mapping_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(active_road_mapping_generate_messages active_road_mapping_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(active_road_mapping_generate_messages_lisp _active_road_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_road_mapping_genlisp)
add_dependencies(active_road_mapping_genlisp active_road_mapping_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_road_mapping_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(active_road_mapping
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_road_mapping
)

### Generating Services

### Generating Module File
_generate_module_nodejs(active_road_mapping
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_road_mapping
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(active_road_mapping_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(active_road_mapping_generate_messages active_road_mapping_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(active_road_mapping_generate_messages_nodejs _active_road_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_road_mapping_gennodejs)
add_dependencies(active_road_mapping_gennodejs active_road_mapping_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_road_mapping_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(active_road_mapping
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_road_mapping
)

### Generating Services

### Generating Module File
_generate_module_py(active_road_mapping
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_road_mapping
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(active_road_mapping_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(active_road_mapping_generate_messages active_road_mapping_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(active_road_mapping_generate_messages_py _active_road_mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_road_mapping_genpy)
add_dependencies(active_road_mapping_genpy active_road_mapping_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_road_mapping_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_road_mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_road_mapping
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(active_road_mapping_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_road_mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_road_mapping
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(active_road_mapping_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_road_mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_road_mapping
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(active_road_mapping_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_road_mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_road_mapping
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(active_road_mapping_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_road_mapping)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_road_mapping\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_road_mapping
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(active_road_mapping_generate_messages_py std_msgs_generate_messages_py)
endif()
