/** moveit编程练习 **/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <math.h>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "test_demo", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.deleteAllMarkers();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  // 连接机械臂实例组  
  moveit::planning_interface::MoveGroupInterface group("xarm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      group.getCurrentState()->getJointModelGroup("xarm");
  // 查看当前的位置
  geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
  ROS_INFO_STREAM("Start pose is : \n"<<start_pose.position.x<<"\n"<<start_pose.position.y<<"\n"<<start_pose.position.z<<std::endl);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  /*** 自定义目标点完成规划  ***/
  // 设置start_pose为当前的位置。
  group.setStartStateToCurrentState();
  // 设置机器人终端的目标位置
  geometry_msgs::Pose target_pose1;
  tf2::Quaternion orientation;

  target_pose1.position.x = 0.2;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.15;
  double yaw = atan2(target_pose1.position.y,target_pose1.position.x);
  orientation.setRPY(0.0, 0.0, yaw);
  target_pose1.orientation = tf2::toMsg(orientation);
  group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    ROS_INFO("--- Plan trajectory Success --- ");
  }else {
    ROS_ERROR("--- Plan trajectory failed --- ");
    exit(1);
  }

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishAxisLabeled(start_pose, "start");
//  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot to target 1");

  // 开始运动规划，并让机械臂移动到目标位置
  group.move();

  geometry_msgs::Pose end_pose = group.getCurrentPose().pose;
  ROS_INFO_STREAM("target pose 1 is : \n"<<end_pose.position.x<<"\n"<<end_pose.position.y<<"\n"<<end_pose.position.z<<std::endl);
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.publishAxisLabeled(end_pose, "goal_1");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the next demo: add object desk");

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.5;
  primitive.dimensions[1] = 1.5;
  primitive.dimensions[2] = 0.01;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = 0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move up 0.15m");

  target_pose1 = group.getCurrentPose().pose;
  /*** 笛卡尔空间坐标，固定轨迹  ***/
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);
  geometry_msgs::Pose goal_pose2;
  goal_pose2 = target_pose1;
  //goal_pose2.position.x = target_pose1.position.x+0.12;
  //goal_pose2.position.y= target_pose1.position.y-0.12;
  goal_pose2.position.z += 0.15;
  waypoints.push_back(goal_pose2);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


//group.execute(trajectory);

  geometry_msgs::Pose end_pose1 = group.getCurrentPose().pose;
  ROS_INFO_STREAM("end pose  is : \n"<<end_pose1.position.x<<"\n"<<end_pose1.position.y<<"\n"<<end_pose1.position.z<<std::endl);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to open gripper");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

   gripper_group.setStartStateToCurrentState();
   std::vector<double> OPEN_GRIPPER_POSE = {0.65,0.65};
   std::vector<double> CLOSE_GRIPPER_POSE = {0.05,0.05};
    gripper_group.setJointValueTarget(OPEN_GRIPPER_POSE);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    success = (gripper_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    if(!success){
      ROS_ERROR("--- Plan trajectory failed --- ");
    }
    ROS_INFO("--- Open gripper! ---");
    gripper_group.move();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to close gripper");
  gripper_group.setJointValueTarget(CLOSE_GRIPPER_POSE);

  success = (gripper_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
  ROS_INFO("--- close gripper! ---");
  gripper_group.move();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to back home");
  std::vector<double> HOME_POSITION = {0, 0, 0, 0, 0, 0};
  group.setStartStateToCurrentState();
  group.setJointValueTarget(HOME_POSITION);
  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("pick place ", "Visualizing reachJointTarget (joint space goal) %s", success ? "" : "FAILED");
  if(success){
    group.move();
  }

  ROS_INFO_STREAM("CTRL C EXIT");
  ros::waitForShutdown();

}
