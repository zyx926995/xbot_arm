# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xarm_pick_place: 0 messages, 5 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Ishape_msgs:/opt/ros/kinetic/share/shape_msgs/cmake/../msg;-Imoveit_msgs:/opt/ros/kinetic/share/moveit_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg;-Iobject_recognition_msgs:/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg;-Ioctomap_msgs:/opt/ros/kinetic/share/octomap_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xarm_pick_place_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv" NAME_WE)
add_custom_target(_xarm_pick_place_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_pick_place" "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv" ""
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv" NAME_WE)
add_custom_target(_xarm_pick_place_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_pick_place" "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv" "moveit_msgs/CollisionObject:std_msgs/Header:geometry_msgs/Quaternion:shape_msgs/SolidPrimitive:geometry_msgs/Point:shape_msgs/Plane:shape_msgs/Mesh:geometry_msgs/Pose:shape_msgs/MeshTriangle:object_recognition_msgs/ObjectType"
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv" NAME_WE)
add_custom_target(_xarm_pick_place_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_pick_place" "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv" ""
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv" NAME_WE)
add_custom_target(_xarm_pick_place_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_pick_place" "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv" ""
)

get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv" NAME_WE)
add_custom_target(_xarm_pick_place_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_pick_place" "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_cpp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/moveit_msgs/cmake/../msg/CollisionObject.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Plane.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Mesh.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/MeshTriangle.msg;/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg/ObjectType.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_cpp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_cpp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_cpp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_pick_place
)

### Generating Module File
_generate_module_cpp(xarm_pick_place
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_pick_place
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xarm_pick_place_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xarm_pick_place_generate_messages xarm_pick_place_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_cpp _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_cpp _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_cpp _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_cpp _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_cpp _xarm_pick_place_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_pick_place_gencpp)
add_dependencies(xarm_pick_place_gencpp xarm_pick_place_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_pick_place_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_eus(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/moveit_msgs/cmake/../msg/CollisionObject.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Plane.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Mesh.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/MeshTriangle.msg;/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg/ObjectType.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_eus(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_eus(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_eus(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_pick_place
)

### Generating Module File
_generate_module_eus(xarm_pick_place
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_pick_place
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xarm_pick_place_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xarm_pick_place_generate_messages xarm_pick_place_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_eus _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_eus _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_eus _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_eus _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_eus _xarm_pick_place_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_pick_place_geneus)
add_dependencies(xarm_pick_place_geneus xarm_pick_place_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_pick_place_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_lisp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/moveit_msgs/cmake/../msg/CollisionObject.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Plane.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Mesh.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/MeshTriangle.msg;/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg/ObjectType.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_lisp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_lisp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_lisp(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_pick_place
)

### Generating Module File
_generate_module_lisp(xarm_pick_place
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_pick_place
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xarm_pick_place_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xarm_pick_place_generate_messages xarm_pick_place_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_lisp _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_lisp _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_lisp _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_lisp _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_lisp _xarm_pick_place_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_pick_place_genlisp)
add_dependencies(xarm_pick_place_genlisp xarm_pick_place_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_pick_place_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_nodejs(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/moveit_msgs/cmake/../msg/CollisionObject.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Plane.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Mesh.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/MeshTriangle.msg;/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg/ObjectType.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_nodejs(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_nodejs(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_nodejs(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_pick_place
)

### Generating Module File
_generate_module_nodejs(xarm_pick_place
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_pick_place
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xarm_pick_place_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xarm_pick_place_generate_messages xarm_pick_place_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_nodejs _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_nodejs _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_nodejs _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_nodejs _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_nodejs _xarm_pick_place_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_pick_place_gennodejs)
add_dependencies(xarm_pick_place_gennodejs xarm_pick_place_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_pick_place_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_py(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/moveit_msgs/cmake/../msg/CollisionObject.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Plane.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/Mesh.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/shape_msgs/cmake/../msg/MeshTriangle.msg;/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg/ObjectType.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_py(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_py(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place
)
_generate_srv_py(xarm_pick_place
  "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place
)

### Generating Module File
_generate_module_py(xarm_pick_place
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xarm_pick_place_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xarm_pick_place_generate_messages xarm_pick_place_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/ArmControl.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_py _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/AddObject.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_py _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPlacePose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_py _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/RemoveObjects.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_py _xarm_pick_place_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xbot/catkin_ws/src/xarm/xarm_pick_place/srv/TargetPickPose.srv" NAME_WE)
add_dependencies(xarm_pick_place_generate_messages_py _xarm_pick_place_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_pick_place_genpy)
add_dependencies(xarm_pick_place_genpy xarm_pick_place_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_pick_place_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_pick_place)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_pick_place
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xarm_pick_place_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(xarm_pick_place_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(xarm_pick_place_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(xarm_pick_place_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET shape_msgs_generate_messages_cpp)
  add_dependencies(xarm_pick_place_generate_messages_cpp shape_msgs_generate_messages_cpp)
endif()
if(TARGET moveit_msgs_generate_messages_cpp)
  add_dependencies(xarm_pick_place_generate_messages_cpp moveit_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_pick_place)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_pick_place
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xarm_pick_place_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(xarm_pick_place_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(xarm_pick_place_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(xarm_pick_place_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET shape_msgs_generate_messages_eus)
  add_dependencies(xarm_pick_place_generate_messages_eus shape_msgs_generate_messages_eus)
endif()
if(TARGET moveit_msgs_generate_messages_eus)
  add_dependencies(xarm_pick_place_generate_messages_eus moveit_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_pick_place)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_pick_place
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xarm_pick_place_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(xarm_pick_place_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(xarm_pick_place_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(xarm_pick_place_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET shape_msgs_generate_messages_lisp)
  add_dependencies(xarm_pick_place_generate_messages_lisp shape_msgs_generate_messages_lisp)
endif()
if(TARGET moveit_msgs_generate_messages_lisp)
  add_dependencies(xarm_pick_place_generate_messages_lisp moveit_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_pick_place)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_pick_place
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xarm_pick_place_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(xarm_pick_place_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(xarm_pick_place_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(xarm_pick_place_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET shape_msgs_generate_messages_nodejs)
  add_dependencies(xarm_pick_place_generate_messages_nodejs shape_msgs_generate_messages_nodejs)
endif()
if(TARGET moveit_msgs_generate_messages_nodejs)
  add_dependencies(xarm_pick_place_generate_messages_nodejs moveit_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_pick_place
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xarm_pick_place_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(xarm_pick_place_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(xarm_pick_place_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(xarm_pick_place_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET shape_msgs_generate_messages_py)
  add_dependencies(xarm_pick_place_generate_messages_py shape_msgs_generate_messages_py)
endif()
if(TARGET moveit_msgs_generate_messages_py)
  add_dependencies(xarm_pick_place_generate_messages_py moveit_msgs_generate_messages_py)
endif()
