/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Bool.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Empty.h>
void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_1_joint";
  posture.joint_names[1] = "gripper_2_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.6;
  posture.points[0].positions[1] = 0.6;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_1_joint";
  posture.joint_names[1] = "gripper_2_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.1;
  posture.points[0].positions[1] = 0.1;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(ros::NodeHandle& nh,moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  ros::Publisher grip_pub = nh.advertise<std_msgs::Bool>("/commands/grip",10);
  ros::Publisher reset_pub = nh.advertise<std_msgs::Empty>("/commands/reset",10);

  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
//  std::vector<moveit_msgs::Grasp> grasps;
//  grasps.resize(1);

//  // Setting grasp pose
//  // ++++++++++++++++++++++
//  grasps[0].grasp_pose.header.frame_id = "base_link";
  move_group.setMaxVelocityScalingFactor(0.7);
  tf2::Quaternion orientation;
  orientation.setRPY(3.1221229319955843, 0.0023293978471917924, 0.07285896372226237);
//  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
//  grasps[0].grasp_pose.pose.position.x = 0.2;
//  grasps[0].grasp_pose.pose.position.y = 0.4;
//  grasps[0].grasp_pose.pose.position.z = 0.32;
//  //move_group.setPositionTarget(0.2,0.4,0.25);

//  move_group.setPoseTarget(grasps[0].grasp_pose);
//  //move_group.setSupportSurfaceName("table1");
//  move_group.move();


  move_group.setStartStateToCurrentState();
  sensor_msgs::JointState joint_state;
  std::vector<std::string> joints_name = { "arm_1_joint", "arm_2_joint", "arm_3_joint",
                                           "arm_4_joint", "arm_5_joint", "arm_6_joint","gripper_1_joint","gripper_2_joint"};

  joint_state.header.stamp = ros::Time::now();

  std::vector<double> joints_value1 = {-0.408691340264, -1.37710207241, -1.63010789462, -1.21323967906, -1.73488993829, 0.146588789105 ,0 ,0};
  joint_state.header.frame_id = "world";
  joint_state.name = joints_name;
  joint_state.position = joints_value1;
  move_group.setJointValueTarget(joint_state);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
  ROS_INFO("--- Plan trajectory Success and start moving --- ");
  // move_group.setPoseTarget(pre_pick_pose);
  move_group.move();
  move_group.getMoveGroupClient().waitForResult();
  if (move_group.getMoveGroupClient().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("You have reached the goal!");
  else{
    ROS_ERROR("The base failed for some reason");
    exit(1);
  }
  std_msgs::Bool msg;
  msg.data = false;
  grip_pub.publish(msg);
  ROS_INFO("GRIP FALSE");
  sleep(1);


  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
  //geometry_msgs::Pose current_pose = grasps[0].grasp_pose.pose;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);

  geometry_msgs::Pose target_pose = current_pose;

  target_pose.position.z = 0.15;
  waypoints.push_back(target_pose);  // down


  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  //move_group.setMaxVelocityScalingFactor(0.35);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory1;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  move_group.execute(trajectory1);
  move_group.getMoveGroupClient().waitForResult();
  if (move_group.getMoveGroupClient().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("You have reached the goal!");
  else{
    ROS_ERROR("The base failed for some reason");
    exit(1);
  }
  sleep(2);
  msg.data = true;
  grip_pub.publish(msg);
  ROS_INFO("GRIP true");
  sleep(2);
  move_group.attachObject("object");




//  //move_group.clearPathConstraints();


  current_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints2;
  waypoints2.push_back(current_pose);
  ROS_ERROR_STREAM(current_pose.position.z);

  geometry_msgs::Pose target_pose2 = current_pose;
  target_pose2.position.z += 0.1;
  ROS_ERROR_STREAM(target_pose2.position.z);

  waypoints2.push_back(target_pose2);  // up

  moveit_msgs::RobotTrajectory trajectory2;
  const double jump_threshold1 = 0.0;
  const double eef_step1 = 0.01;

  double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step1, jump_threshold1, trajectory2);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction2 * 100.0);
  move_group.execute(trajectory2);
  move_group.getMoveGroupClient().waitForResult();
  if (move_group.getMoveGroupClient().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("You have reached the goal!");
  else{
    ROS_ERROR("The base failed for some reason");
    exit(1);
  }


  joints_value1 = move_group.getCurrentJointValues();
  joints_value1[0] = -0.9;
  joints_value1[5] -= 0.1;
  joint_state.position = joints_value1;
  move_group.setJointValueTarget(joint_state);


  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
  ROS_INFO("--- Plan trajectory Success and start moving --- ");
  // move_group.setPoseTarget(pre_pick_pose);
  move_group.move();
  move_group.getMoveGroupClient().waitForResult();
  if (move_group.getMoveGroupClient().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("You have reached the goal!");
  else{
    ROS_ERROR("The base failed for some reason");
    exit(1);
  }

  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
  target_pose3.position.z = 0.15;

  move_group.setPoseTarget(target_pose3);

  move_group.move();
  if (move_group.getMoveGroupClient().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("You have reached the goal!");
  else{
    ROS_ERROR("The base failed for some reason");
    exit(1);
  }
  msg.data = false;
  grip_pub.publish(msg);
  ROS_INFO("GRIP FALSE");
  sleep(2);
  move_group.detachObject("object");
  sleep(1);

  current_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints3;
  waypoints3.push_back(current_pose);
  ROS_ERROR_STREAM(current_pose.position.z);

  geometry_msgs::Pose target_pose4 = current_pose;
//  target_pose4.orientation = tf2::toMsg(orientation);
  target_pose4.position.z += 0.15;
  ROS_ERROR_STREAM(target_pose4.position.z);

  waypoints3.push_back(target_pose4);  // up



  moveit_msgs::RobotTrajectory trajectory3;

  double fraction3= move_group.computeCartesianPath(waypoints3, eef_step1, jump_threshold1, trajectory3);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction3 * 100.0);
  move_group.execute(trajectory3);
  if (move_group.getMoveGroupClient().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("You have reached the goal!");
  else{
    ROS_ERROR("The base failed for some reason");
    exit(1);
  }

  ROS_INFO("111");
  std::vector<std::string> object_ids;
  object_ids.push_back("table1");

  object_ids.push_back("object");
  planning_scene_interface.removeCollisionObjects(object_ids);

  msg.data = true;
  grip_pub.publish(msg);
  ROS_INFO("GRIP true");
  sleep(1);

  joints_value1 = {0, 0, 0, 0, 0, 0 ,0 ,0};
  joint_state.position = joints_value1;
  move_group.setJointValueTarget(joint_state);

   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
  ROS_INFO("--- Plan trajectory Success and start moving --- ");
  // move_group.setPoseTarget(pre_pick_pose);
  move_group.move();



}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 2 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.2;
  collision_objects[0].primitives[0].dimensions[1] = 1.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.01;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.005;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;


  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "base_link";
  collision_objects[1].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.01;
  collision_objects[1].primitives[0].dimensions[1] = 0.01;
  collision_objects[1].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.2;
  collision_objects[1].primitive_poses[0].position.y = 0.4;
  collision_objects[1].primitive_poses[0].position.z = 0.11;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(6);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("xarm");
  group.setPlanningTime(45.0);

  // 把桌面和被抓取的物体加进planning_scene
  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(nh,group,planning_scene_interface);

  ros::WallDuration(1.0).sleep();

  //place(group);

  ros::waitForShutdown();
  return 0;
}

// BEGIN_TUTORIAL
// CALL_SUB_TUTORIAL table1
// CALL_SUB_TUTORIAL table2
// CALL_SUB_TUTORIAL object
//
// Pick Pipeline
// ^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL pick1
// openGripper function
// """"""""""""""""""""
// CALL_SUB_TUTORIAL open_gripper
// CALL_SUB_TUTORIAL pick2
// closedGripper function
// """"""""""""""""""""""
// CALL_SUB_TUTORIAL closed_gripper
// CALL_SUB_TUTORIAL pick3
//
// Place Pipeline
// ^^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL place
// END_TUTORIAL
