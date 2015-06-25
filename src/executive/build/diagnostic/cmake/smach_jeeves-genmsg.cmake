# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "smach_jeeves: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ismach_jeeves:/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(smach_jeeves_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Feedback.msg" NAME_WE)
add_custom_target(_smach_jeeves_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "smach_jeeves" "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Feedback.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Status.msg" NAME_WE)
add_custom_target(_smach_jeeves_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "smach_jeeves" "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Status.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/LaserScan.msg" NAME_WE)
add_custom_target(_smach_jeeves_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "smach_jeeves" "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/LaserScan.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/smach_jeeves
)
_generate_msg_cpp(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/smach_jeeves
)
_generate_msg_cpp(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/smach_jeeves
)

### Generating Services

### Generating Module File
_generate_module_cpp(smach_jeeves
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/smach_jeeves
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(smach_jeeves_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(smach_jeeves_generate_messages smach_jeeves_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Feedback.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_cpp _smach_jeeves_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Status.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_cpp _smach_jeeves_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/LaserScan.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_cpp _smach_jeeves_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(smach_jeeves_gencpp)
add_dependencies(smach_jeeves_gencpp smach_jeeves_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS smach_jeeves_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/smach_jeeves
)
_generate_msg_lisp(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/smach_jeeves
)
_generate_msg_lisp(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/smach_jeeves
)

### Generating Services

### Generating Module File
_generate_module_lisp(smach_jeeves
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/smach_jeeves
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(smach_jeeves_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(smach_jeeves_generate_messages smach_jeeves_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Feedback.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_lisp _smach_jeeves_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Status.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_lisp _smach_jeeves_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/LaserScan.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_lisp _smach_jeeves_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(smach_jeeves_genlisp)
add_dependencies(smach_jeeves_genlisp smach_jeeves_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS smach_jeeves_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/smach_jeeves
)
_generate_msg_py(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/smach_jeeves
)
_generate_msg_py(smach_jeeves
  "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/smach_jeeves
)

### Generating Services

### Generating Module File
_generate_module_py(smach_jeeves
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/smach_jeeves
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(smach_jeeves_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(smach_jeeves_generate_messages smach_jeeves_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Feedback.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_py _smach_jeeves_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/Status.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_py _smach_jeeves_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sr/catkin_make/roboticsclub-mcecsbot/src/executive/src/diagnostic/msg/LaserScan.msg" NAME_WE)
add_dependencies(smach_jeeves_generate_messages_py _smach_jeeves_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(smach_jeeves_genpy)
add_dependencies(smach_jeeves_genpy smach_jeeves_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS smach_jeeves_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/smach_jeeves)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/smach_jeeves
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(smach_jeeves_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/smach_jeeves)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/smach_jeeves
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(smach_jeeves_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/smach_jeeves)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/smach_jeeves\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/smach_jeeves
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(smach_jeeves_generate_messages_py std_msgs_generate_messages_py)
