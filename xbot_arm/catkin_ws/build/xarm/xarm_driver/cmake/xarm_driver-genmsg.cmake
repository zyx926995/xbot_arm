# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xarm_driver: 3 messages, 3 services")

set(MSG_I_FLAGS "-Ixarm_driver:/home/xbot/catkin_ws/src/xarm/xarm_driver/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xarm_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg" NAME_WE)
add_custom_target(_xarm_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_driver" "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg" ""
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv" NAME_WE)
add_custom_target(_xarm_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_driver" "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv" "std_msgs/Empty"
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg" NAME_WE)
add_custom_target(_xarm_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_driver" "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv" NAME_WE)
add_custom_target(_xarm_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_driver" "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg" NAME_WE)
add_custom_target(_xarm_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_driver" "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg" ""
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv" NAME_WE)
add_custom_target(_xarm_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_driver" "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv" "sensor_msgs/JointState:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver
)
_generate_msg_cpp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver
)
_generate_msg_cpp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver
)

### Generating Services
_generate_srv_cpp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver
)
_generate_srv_cpp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver
)
_generate_srv_cpp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Empty.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver
)

### Generating Module File
_generate_module_cpp(xarm_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xarm_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xarm_driver_generate_messages xarm_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_cpp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_cpp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_cpp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_cpp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_cpp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_cpp _xarm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_driver_gencpp)
add_dependencies(xarm_driver_gencpp xarm_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver
)
_generate_msg_eus(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver
)
_generate_msg_eus(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver
)

### Generating Services
_generate_srv_eus(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver
)
_generate_srv_eus(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver
)
_generate_srv_eus(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Empty.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver
)

### Generating Module File
_generate_module_eus(xarm_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xarm_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xarm_driver_generate_messages xarm_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_eus _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_eus _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_eus _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_eus _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_eus _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_eus _xarm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_driver_geneus)
add_dependencies(xarm_driver_geneus xarm_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver
)
_generate_msg_lisp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver
)
_generate_msg_lisp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver
)

### Generating Services
_generate_srv_lisp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver
)
_generate_srv_lisp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver
)
_generate_srv_lisp(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Empty.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver
)

### Generating Module File
_generate_module_lisp(xarm_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xarm_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xarm_driver_generate_messages xarm_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_lisp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_lisp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_lisp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_lisp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_lisp _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_lisp _xarm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_driver_genlisp)
add_dependencies(xarm_driver_genlisp xarm_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver
)
_generate_msg_nodejs(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver
)
_generate_msg_nodejs(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver
)

### Generating Services
_generate_srv_nodejs(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver
)
_generate_srv_nodejs(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver
)
_generate_srv_nodejs(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Empty.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver
)

### Generating Module File
_generate_module_nodejs(xarm_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xarm_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xarm_driver_generate_messages xarm_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_nodejs _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_nodejs _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_nodejs _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_nodejs _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_nodejs _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_nodejs _xarm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_driver_gennodejs)
add_dependencies(xarm_driver_gennodejs xarm_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver
)
_generate_msg_py(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver
)
_generate_msg_py(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver
)

### Generating Services
_generate_srv_py(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver
)
_generate_srv_py(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver
)
_generate_srv_py(xarm_driver
  "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Empty.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver
)

### Generating Module File
_generate_module_py(xarm_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xarm_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xarm_driver_generate_messages xarm_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/JointLocation.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_py _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CallVersion.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_py _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/SingleJointControl.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_py _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandPose.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_py _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/msg/MotorStatus.msg" NAME_WE)
add_dependencies(xarm_driver_generate_messages_py _xarm_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_driver/srv/CommandJoint.srv" NAME_WE)
add_dependencies(xarm_driver_generate_messages_py _xarm_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_driver_genpy)
add_dependencies(xarm_driver_genpy xarm_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xarm_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(xarm_driver_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(xarm_driver_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(xarm_driver_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xarm_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(xarm_driver_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(xarm_driver_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(xarm_driver_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xarm_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(xarm_driver_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(xarm_driver_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(xarm_driver_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xarm_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(xarm_driver_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(xarm_driver_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(xarm_driver_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xarm_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(xarm_driver_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(xarm_driver_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(xarm_driver_generate_messages_py sensor_msgs_generate_messages_py)
endif()
