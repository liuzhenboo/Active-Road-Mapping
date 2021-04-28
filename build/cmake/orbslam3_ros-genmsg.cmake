# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "orbslam3_ros: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iorbslam3_ros:/home/lzb/ros_ws/src/Active-Road-Mapping/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(orbslam3_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_custom_target(_orbslam3_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orbslam3_ros" "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(orbslam3_ros
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam3_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(orbslam3_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam3_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(orbslam3_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(orbslam3_ros_generate_messages orbslam3_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(orbslam3_ros_generate_messages_cpp _orbslam3_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orbslam3_ros_gencpp)
add_dependencies(orbslam3_ros_gencpp orbslam3_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orbslam3_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(orbslam3_ros
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orbslam3_ros
)

### Generating Services

### Generating Module File
_generate_module_eus(orbslam3_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orbslam3_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(orbslam3_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(orbslam3_ros_generate_messages orbslam3_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(orbslam3_ros_generate_messages_eus _orbslam3_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orbslam3_ros_geneus)
add_dependencies(orbslam3_ros_geneus orbslam3_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orbslam3_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(orbslam3_ros
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam3_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(orbslam3_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam3_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(orbslam3_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(orbslam3_ros_generate_messages orbslam3_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(orbslam3_ros_generate_messages_lisp _orbslam3_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orbslam3_ros_genlisp)
add_dependencies(orbslam3_ros_genlisp orbslam3_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orbslam3_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(orbslam3_ros
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/orbslam3_ros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(orbslam3_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/orbslam3_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(orbslam3_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(orbslam3_ros_generate_messages orbslam3_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(orbslam3_ros_generate_messages_nodejs _orbslam3_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orbslam3_ros_gennodejs)
add_dependencies(orbslam3_ros_gennodejs orbslam3_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orbslam3_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(orbslam3_ros
  "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam3_ros
)

### Generating Services

### Generating Module File
_generate_module_py(orbslam3_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam3_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(orbslam3_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(orbslam3_ros_generate_messages orbslam3_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzb/ros_ws/src/Active-Road-Mapping/msg/Transform_matrix.msg" NAME_WE)
add_dependencies(orbslam3_ros_generate_messages_py _orbslam3_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orbslam3_ros_genpy)
add_dependencies(orbslam3_ros_genpy orbslam3_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orbslam3_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam3_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam3_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(orbslam3_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orbslam3_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orbslam3_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(orbslam3_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam3_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam3_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(orbslam3_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/orbslam3_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/orbslam3_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(orbslam3_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam3_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam3_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam3_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(orbslam3_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
