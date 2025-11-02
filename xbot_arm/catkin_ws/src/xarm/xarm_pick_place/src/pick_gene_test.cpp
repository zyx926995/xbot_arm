#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <iostream>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>






int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_xarm");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(6);
  spinner.start();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group("xarm");
  moveit::planning_interface::MoveGroupInterface grip_group("gripper");
  ROS_INFO("pick and place test!");


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  arm_group.setStartStateToCurrentState();
  double yaw = 0;
  geometry_msgs::Pose target_pose;

//  arm_group.setPositionTarget(0.1,0.35,0.28);



//  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  arm_group.move();

 //这是z轴向下
  for(int i = 0;i<=180;i+=1){
  tf2::Quaternion orientation;
  double rad = i/180.0*3.1415926;
  orientation.setRPY(3.1415926, 0, rad);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.4;
  target_pose.position.z = 0.28;

  arm_group.setPoseTarget(target_pose);


  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  std::cout<<rad <<" i ="<<i<<std::endl;
  if(success){
    arm_group.move();
    sensor_msgs::JointState joint_state;
    std::vector<std::string> joints_name = {"gripper_1_joint","gripper_2_joint"};

    joint_state.header.stamp = ros::Time::now();

    std::vector<double> joints_value1 = {0.6,0.6};
    joint_state.header.frame_id = "world";
    joint_state.name = joints_name;
    joint_state.position = joints_value1;
    grip_group.setJointValueTarget(joint_state);
    grip_group.move();

    arm_group.setStartStateToCurrentState();

    geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
    //geometry_msgs::Pose current_pose = grasps[0].grasp_pose.pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose);

    geometry_msgs::Pose target_pose1 = current_pose;

    target_pose1.position.z -= 0.12;
    waypoints.push_back(target_pose1);  // down


    moveit_msgs::RobotTrajectory trajectory1;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    if(abs(fraction - 1.0)<=0.01){
      arm_group.execute(trajectory1);
      break;
    }else {
      continue;
//      sensor_msgs::JointState joint_state;
//      std::vector<std::string> joints_name = { "arm_1_joint", "arm_2_joint", "arm_3_joint",
//                                               "arm_4_joint", "arm_5_joint", "arm_6_joint","gripper_1_joint","gripper_2_joint"};

//      joint_state.header.stamp = ros::Time::now();

//      std::vector<double> joints_value1 = {0, 0, 0, 0, 0, 0 ,0 ,0};
//      joint_state.header.frame_id = "world";
//      joint_state.name = joints_name;
//      joint_state.position = joints_value1;
//      arm_group.setJointValueTarget(joint_state);
//      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//      bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//      ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//      if(!success){
//        ROS_ERROR("--- Plan trajectory failed --- ");
//      }
//      ROS_INFO("--- Plan trajectory Success and start moving --- ");
//      // move_group.setPoseTarget(pre_pick_pose);
//      arm_group.move();


    }

  }
  }

//  for(int i = 0;i<12;i++){
//    tf2::Quaternion orientation;
//    orientation.setRPY(0, 0, -3.1415926);
//    target_pose.orientation = tf2::toMsg(orientation);
//    target_pose.position.x = 0.2;
//    target_pose.position.y = 0.28+i/100.0;
//    target_pose.position.z = 0.2;

//    arm_group.setPoseTarget(target_pose);


//    bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//      if(success){
//        ROS_INFO_STREAM(target_pose.position.y);
//        break;

//      }

// }
  ROS_INFO("--- Plan trajectory Success and start moving --- ");
  // move_group.setPoseTarget(pre_pick_pose);






  ros::waitForShutdown();
  return 0;
}
