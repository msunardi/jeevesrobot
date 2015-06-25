# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "diagnostics: 3 messages, 0 services")

set(MSG_I_FLAGS "-Idiagnostics:/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(diagnostics_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/LaserScan.msg" NAME_WE)
add_custom_target(_diagnostics_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diagnostics" "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/LaserScan.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Status.msg" NAME_WE)
add_custom_target(_diagnostics_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diagnostics" "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Status.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Feedback.msg" NAME_WE)
add_custom_target(_diagnostics_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diagnostics" "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Feedback.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diagnostics
)
_generate_msg_cpp(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diagnostics
)
_generate_msg_cpp(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diagnostics
)

### Generating Services

### Generating Module File
_generate_module_cpp(diagnostics
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diagnostics
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(diagnostics_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(diagnostics_generate_messages diagnostics_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/LaserScan.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_cpp _diagnostics_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Status.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_cpp _diagnostics_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Feedback.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_cpp _diagnostics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diagnostics_gencpp)
add_dependencies(diagnostics_gencpp diagnostics_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diagnostics_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diagnostics
)
_generate_msg_lisp(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diagnostics
)
_generate_msg_lisp(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diagnostics
)

### Generating Services

### Generating Module File
_generate_module_lisp(diagnostics
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diagnostics
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(diagnostics_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(diagnostics_generate_messages diagnostics_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/LaserScan.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_lisp _diagnostics_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Status.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_lisp _diagnostics_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Feedback.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_lisp _diagnostics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diagnostics_genlisp)
add_dependencies(diagnostics_genlisp diagnostics_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diagnostics_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diagnostics
)
_generate_msg_py(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diagnostics
)
_generate_msg_py(diagnostics
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diagnostics
)

### Generating Services

### Generating Module File
_generate_module_py(diagnostics
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diagnostics
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(diagnostics_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(diagnostics_generate_messages diagnostics_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/LaserScan.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_py _diagnostics_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Status.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_py _diagnostics_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostics/msg/Feedback.msg" NAME_WE)
add_dependencies(diagnostics_generate_messages_py _diagnostics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diagnostics_genpy)
add_dependencies(diagnostics_genpy diagnostics_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diagnostics_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diagnostics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diagnostics
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(diagnostics_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diagnostics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diagnostics
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(diagnostics_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diagnostics)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diagnostics\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diagnostics
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(diagnostics_generate_messages_py std_msgs_generate_messages_py)
