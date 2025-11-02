cmake_minimum_required(VERSION 2.8.3)
project(xarm_pick_place)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  #moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools
  shape_msgs
  moveit_msgs
  xarm_driver
  #baxter_core_msgs
  std_msgs
  #geometry_msgs
  #baxter_control
  roscpp
  cmake_modules
)

#find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   AddObject.srv
   TargetPickPose.srv
   TargetPlacePose.srv
   RemoveObjects.srv
   ArmControl.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  shape_msgs
  moveit_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

# Enable gdb to show line numbers
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
#set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(server_pick_place src/pick_place_server.cpp)
add_dependencies(server_pick_place xarm_pick_place_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(server_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Main executable
#add_executable(generate_pick_place src/pick_place_generate.cpp)
#target_link_libraries(generate_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#add_executable(block_pick_place src/block_pick_place.cpp)
#target_link_libraries(block_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(xarm_pick_place src/xarm_pick_place.cpp)
#target_link_libraries(xarm_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(server_pick_place src/pick_place_server.cpp)
#target_link_libraries(server_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(client_pick_place src/pick_place_client.cpp)
add_dependencies(client_pick_place xarm_pick_place_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(client_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(excute_pick_place src/pick_place_demo.cpp)
#add_dependencies(excute_pick_place xarm_pick_place_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(excute_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(pick_gene_test src/pick_gene_test.cpp)
#target_link_libraries(pick_gene_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS server_pick_place
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS client_pick_place
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
