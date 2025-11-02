cmake_minimum_required(VERSION 2.8.3)
project(xarm_tutorials)

add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    xarm_driver
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)


#catkin_package(
#  #INCLUDE_DIRS include
#  LIBRARIES xbot_arm_tutorials
#  CATKIN_DEPENDS moveit_core moveit_visual_tools moveit_ros_planning_interface interactive_markers
#  DEPENDS system_lib
#  #EIGEN3

#)

############
### Build ##
############

include_directories(
  #include
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS}
)
#add_executable(move_group_interface_test src/move_group_interface_tutorial.cpp)
#target_link_libraries(move_group_interface_test ${catkin_LIBRARIES} )
#add_dependencies(move_group_interface_test ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(demo_test src/test_demo.cpp)
target_link_libraries(demo_test ${catkin_LIBRARIES} )
add_dependencies(demo_test ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(greet_demo src/greet_demo.cpp)
target_link_libraries(greet_demo ${catkin_LIBRARIES} )
add_dependencies(greet_demo ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS greet_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




