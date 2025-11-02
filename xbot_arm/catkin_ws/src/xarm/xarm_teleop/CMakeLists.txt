################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(xarm_teleop)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    #open_manipulator_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    geometry_msgs
    xarm_driver
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(xarm_teleop_keyboard src/xbot_arm_teleop_keyboard.cpp)
add_dependencies(xarm_teleop_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xarm_teleop_keyboard ${catkin_LIBRARIES} )


add_executable(simple_keyboard src/simple_keyboard.cpp)
add_dependencies(simple_keyboard xarm_driver_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(simple_keyboard ${catkin_LIBRARIES} )

#add_executable(xarm_teleop_joystick src/xarm_teleop_joystick.cpp)
#add_dependencies(xarm_teleop_joystick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(xarm_teleop_joystick ${catkin_LIBRARIES} )

################################################################################
# Install
################################################################################


install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS xarm_teleop_keyboard
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
