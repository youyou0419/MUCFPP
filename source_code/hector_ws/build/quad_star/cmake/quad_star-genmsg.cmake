# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "quad_star: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(quad_star_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv" NAME_WE)
add_custom_target(_quad_star_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "quad_star" "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(quad_star
  "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/quad_star
)

### Generating Module File
_generate_module_cpp(quad_star
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/quad_star
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(quad_star_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(quad_star_generate_messages quad_star_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv" NAME_WE)
add_dependencies(quad_star_generate_messages_cpp _quad_star_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(quad_star_gencpp)
add_dependencies(quad_star_gencpp quad_star_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS quad_star_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(quad_star
  "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/quad_star
)

### Generating Module File
_generate_module_eus(quad_star
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/quad_star
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(quad_star_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(quad_star_generate_messages quad_star_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv" NAME_WE)
add_dependencies(quad_star_generate_messages_eus _quad_star_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(quad_star_geneus)
add_dependencies(quad_star_geneus quad_star_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS quad_star_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(quad_star
  "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/quad_star
)

### Generating Module File
_generate_module_lisp(quad_star
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/quad_star
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(quad_star_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(quad_star_generate_messages quad_star_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv" NAME_WE)
add_dependencies(quad_star_generate_messages_lisp _quad_star_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(quad_star_genlisp)
add_dependencies(quad_star_genlisp quad_star_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS quad_star_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(quad_star
  "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/quad_star
)

### Generating Module File
_generate_module_nodejs(quad_star
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/quad_star
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(quad_star_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(quad_star_generate_messages quad_star_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv" NAME_WE)
add_dependencies(quad_star_generate_messages_nodejs _quad_star_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(quad_star_gennodejs)
add_dependencies(quad_star_gennodejs quad_star_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS quad_star_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(quad_star
  "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/quad_star
)

### Generating Module File
_generate_module_py(quad_star
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/quad_star
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(quad_star_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(quad_star_generate_messages quad_star_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zya/hector_ws/src/quad_star/srv/PathPlan.srv" NAME_WE)
add_dependencies(quad_star_generate_messages_py _quad_star_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(quad_star_genpy)
add_dependencies(quad_star_genpy quad_star_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS quad_star_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/quad_star)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/quad_star
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(quad_star_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(quad_star_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/quad_star)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/quad_star
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(quad_star_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(quad_star_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/quad_star)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/quad_star
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(quad_star_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(quad_star_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/quad_star)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/quad_star
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(quad_star_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(quad_star_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/quad_star)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/quad_star\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/quad_star
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(quad_star_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(quad_star_generate_messages_py geometry_msgs_generate_messages_py)
endif()
