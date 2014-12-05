# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "slam_sensors: 1 messages, 0 services")

set(MSG_I_FLAGS "-Islam_sensors:/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(slam_sensors_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg/RobotVelocity.msg" NAME_WE)
add_custom_target(_slam_sensors_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "slam_sensors" "/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg/RobotVelocity.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(slam_sensors
  "/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg/RobotVelocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slam_sensors
)

### Generating Services

### Generating Module File
_generate_module_cpp(slam_sensors
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slam_sensors
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(slam_sensors_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(slam_sensors_generate_messages slam_sensors_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg/RobotVelocity.msg" NAME_WE)
add_dependencies(slam_sensors_generate_messages_cpp _slam_sensors_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(slam_sensors_gencpp)
add_dependencies(slam_sensors_gencpp slam_sensors_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS slam_sensors_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(slam_sensors
  "/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg/RobotVelocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slam_sensors
)

### Generating Services

### Generating Module File
_generate_module_lisp(slam_sensors
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slam_sensors
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(slam_sensors_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(slam_sensors_generate_messages slam_sensors_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg/RobotVelocity.msg" NAME_WE)
add_dependencies(slam_sensors_generate_messages_lisp _slam_sensors_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(slam_sensors_genlisp)
add_dependencies(slam_sensors_genlisp slam_sensors_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS slam_sensors_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(slam_sensors
  "/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg/RobotVelocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slam_sensors
)

### Generating Services

### Generating Module File
_generate_module_py(slam_sensors
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slam_sensors
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(slam_sensors_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(slam_sensors_generate_messages slam_sensors_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jonghyunahn/SchoolWork/EE/EE125/project/fedora-slambot/src/slam_sensors/msg/RobotVelocity.msg" NAME_WE)
add_dependencies(slam_sensors_generate_messages_py _slam_sensors_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(slam_sensors_genpy)
add_dependencies(slam_sensors_genpy slam_sensors_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS slam_sensors_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slam_sensors)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/slam_sensors
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(slam_sensors_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(slam_sensors_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slam_sensors)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/slam_sensors
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(slam_sensors_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(slam_sensors_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slam_sensors)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slam_sensors\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/slam_sensors
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(slam_sensors_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(slam_sensors_generate_messages_py std_msgs_generate_messages_py)
