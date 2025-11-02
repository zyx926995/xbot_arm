/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/


#include "xbot_arm_teleop_keyboard.h"

XbotArmTeleop::XbotArmTeleop()
    :node_handle_(""),
     priv_node_handle_("~")
{

  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  initSubscriber();

  ROS_INFO("Xbot Arm initialization");
}

XbotArmTeleop::~XbotArmTeleop()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}



bool XbotArmTeleop::openGripper( moveit::planning_interface::MoveGroupInterface & gripper_group){
  sensor_msgs::JointState joint_state;
  gripper_group.setStartStateToCurrentState();

  gripper_group.setJointValueTarget(OPEN_GRIPPER_POSE);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
    return false;
  }
  ROS_INFO("--- Open gripper! ---");
  gripper_group.move();
  return true;

}


bool XbotArmTeleop::closeGripper( moveit::planning_interface::MoveGroupInterface & gripper_group){
  sensor_msgs::JointState joint_state;
  gripper_group.setStartStateToCurrentState();

  gripper_group.setJointValueTarget(CLOSE_GRIPPER_POSE);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
    return false;
  }
  ROS_INFO("--- Close gripper! ---");
  gripper_group.move();
  return true;

}

bool XbotArmTeleop::moveSingleArm(moveit::planning_interface::MoveGroupInterface & arm_group, int index, double joint_position){
  const robot_state::JointModelGroup* joint_model_group =
      arm_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state = arm_group.getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[index] = joint_position;  // radians
  arm_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan  (moveSingleArm) %s", success ? "" : "FAILED");
  if(success){
    arm_group.move();
    return true;
  }
  return false;
}


bool XbotArmTeleop::reachJointTarget(moveit::planning_interface::MoveGroupInterface & arm_group,std::vector<double> pose){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  arm_group.setStartStateToCurrentState();
  arm_group.setJointValueTarget(pose);
  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  if(success){
    arm_group.move();
    return true;
  }
  return false;
}



void XbotArmTeleop::initSubscriber()
{
  grip_pub_ = node_handle_.advertise<std_msgs::Bool>("/commands/grip",10);

}

//void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
//{
//  std::vector<double> temp_angle;
//  temp_angle.resize(NUM_OF_JOINT);
//  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
//  {
//    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
//    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
//    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
//    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
//  }
//  present_joint_angle_ = temp_angle;

//}



std::vector<double> XbotArmTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}
std::vector<double> XbotArmTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

void XbotArmTeleop::printText(moveit::planning_interface::MoveGroupInterface& arm_group)
{
  //moveit::planning_interface::MoveGroupInterface arm_group("xarm");
  joints_value_ = arm_group.getCurrentJointValues();
  goal_pose = arm_group.getCurrentPose().pose;
  joint_state_.header.frame_id = "base_footprint";
  joint_state_.name = joints_name_;
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your Xbot-Arm!\n");
  printf("---------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("r : increase joint 1 angle\n");
  printf("f : decrease joint 1 angle\n");
  printf("t : increase joint 2 angle\n");
  printf("g : decrease joint 2 angle\n");
  printf("y : increase joint 3 angle\n");
  printf("h : decrease joint 3 angle\n");
  printf("u : increase joint 4 angle\n");
  printf("j : decrease joint 4 angle\n");
  printf("i : increase joint 5 angle\n");
  printf("k : decrease joint 5 angle\n");
  printf("o : increase joint 6 angle\n");
  printf("l : decrease joint 6 angle\n");
  printf("\n");
  printf("n : gripper open\n");
  printf("m : gripper close\n");
  printf("       \n");
  printf("1 : init pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");

  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf J5: %.3lf J6: %.3lf\n",
         joints_value_[0],
         joints_value_[1],
         joints_value_[2],
         joints_value_[3],
         joints_value_[4],
         joints_value_[5]);
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         goal_pose.position.x,
         goal_pose.position.y,
         goal_pose.position.z);
  printf("---------------------------\n");

}

void XbotArmTeleop::setGoal(char ch,moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface & gripper_group)
{
  std::string plan_frame = arm_group.getPlanningFrame();
  ROS_INFO_STREAM("Plan frame is : "<<plan_frame);

  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(6, 0.0);

  joints_value_ = arm_group.getCurrentJointValues();
  if(ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goal_pose = arm_group.getCurrentPose().pose;
    goal_pose.position.x += POSE_DELTA;
    arm_group.setPoseTarget(goal_pose);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goal_pose = arm_group.getCurrentPose().pose;
    goal_pose.position.x -= POSE_DELTA;
    arm_group.setPoseTarget(goal_pose);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goal_pose = arm_group.getCurrentPose().pose;
    goal_pose.position.y += POSE_DELTA;
    //ROS_ERROR_STREAM(goal_pose.position.x<<goal_pose.position.y<<goal_pose.position.z);
    //arm_group.setPositionTarget(goal_pose.position.x,goal_pose.position.y,goal_pose.position.z);
    arm_group.setPoseTarget(goal_pose);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'd' || ch == 'D')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goal_pose = arm_group.getCurrentPose().pose;
    goal_pose.position.y -= POSE_DELTA;
    //ROS_ERROR_STREAM(goal_pose.position.x<<goal_pose.position.y<<goal_pose.position.z);
    //arm_group.setPositionTarget(goal_pose.position.x,goal_pose.position.y,goal_pose.position.z);
    arm_group.setPoseTarget(goal_pose);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'z' || ch == 'Z')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goal_pose = arm_group.getCurrentPose().pose;
    goal_pose.position.z += POSE_DELTA;
    //ROS_ERROR_STREAM(goal_pose.position.x<<goal_pose.position.y<<goal_pose.position.z);
    //arm_group.setPositionTarget(goal_pose.position.x,goal_pose.position.y,goal_pose.position.z);
    arm_group.setPoseTarget(goal_pose);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goal_pose = arm_group.getCurrentPose().pose;
    goal_pose.position.z -= POSE_DELTA;
    //ROS_ERROR_STREAM(goal_pose.position.x<<goal_pose.position.y<<goal_pose.position.z);
    //arm_group.setPositionTarget(goal_pose.position.x,goal_pose.position.y,goal_pose.position.z);
    arm_group.setPoseTarget(goal_pose);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'r' || ch == 'R')
  {
    printf("input : r \tincrease(++) joint 1 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[0] += JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'f' || ch == 'F')
  {
    printf("input : f \tdecrease(++) joint 1 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[0] -= JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");

  }
  else if(ch == 't' || ch == 'T')
  {
    printf("input : t \tdincrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[1] += JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }

  else if(ch == 'g' || ch == 'G')
  {
    printf("input : G \tdecrease(++) joint 2 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[1] -= JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 3 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[2] += JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 3 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[2] -= JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }

  else if(ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 4 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[3] += JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 4 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[3] -= JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }

  else if(ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 5 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[4] += JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 5 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[4] -= JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }

  else if(ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 6 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[5] += JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }
  else if(ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 6 angle\n");
    joint_state_.header.stamp = ros::Time::now();
    joints_value_[5] -= JOINT_DELTA;
    joint_state_.position = joints_value_;
    arm_group.setJointValueTarget(joint_state_);
    arm_group.move();
    ROS_INFO("Finish");
  }

  else if(ch == 'N' || ch == 'n')
  {
    printf("input : g \topen gripper\n");
    std_msgs::Bool msg;
    msg.data = false;
    //grip_pub_.publish(msg);
    openGripper(gripper_group);
    ROS_INFO("Gripper open");
  }
  else if(ch == 'm' || ch == 'M')
  {
    printf("input : f \tclose gripper\n");
    std_msgs::Bool msg;
    msg.data = true;
    //grip_pub_.publish(msg);
    closeGripper(gripper_group);
    ROS_INFO("Gripper close");
  }

  else if(ch == '1')
  {
    reachJointTarget(arm_group,HOME_POSITION);
    printf("input : 1 \tinit pose\n");
  }
}

void XbotArmTeleop::restoreTerminalSettings(void)
{
    tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void XbotArmTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_teleop");

  XbotArmTeleop arm_teleop;

  ROS_INFO("Xbot Arm teleoperation using keyboard start");
  arm_teleop.disableWaitingForEnter();
  ros::NodeHandle nh;

  //ros::spinOnce();
  ros::AsyncSpinner spinner(6);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm_group("xarm");
  moveit::planning_interface::MoveGroupInterface gripper_group(GRIPPER_PLANNING_GROUP);

  arm_teleop.printText(arm_group);

  char ch;
  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    ros::spinOnce();
    arm_teleop.printText(arm_group);
    ros::spinOnce();
    arm_teleop.setGoal(ch,arm_group,gripper_group);
  }

  printf("input : q \tTeleop. is finished\n");
  arm_teleop.restoreTerminalSettings();

  return 0;
}
