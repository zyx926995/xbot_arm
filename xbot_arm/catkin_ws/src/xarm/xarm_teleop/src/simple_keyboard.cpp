
#include "simple_keyboard.h"
#include "xarm_driver/SingleJointControl.h"
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



void XbotArmTeleop::initSubscriber()
{
  grip_pub_ = node_handle_.advertise<std_msgs::Bool>("arm/commands/grip",10);
  single_arm_pub_ = node_handle_.advertise<xarm_driver::SingleJointControl>("arm/commands/single_joint_control",10);
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &XbotArmTeleop::jointStatesCallback, this);

}

void XbotArmTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(int i = 0; i < msg->name.size(); i ++)
  {

    if(!msg->name.at(i).compare("arm_1_joint"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("arm_2_joint"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("arm_3_joint"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("arm_4_joint"))  temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("arm_5_joint"))  temp_angle.at(4) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("arm_6_joint"))  temp_angle.at(5) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("gripper_1_joint"))  temp_angle.at(6) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("gripper_2_joint"))  temp_angle.at(7) = (msg->position.at(i));

  }
  joints_value_ = temp_angle;

}



std::vector<double> XbotArmTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}
std::vector<double> XbotArmTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

void XbotArmTeleop::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your Xbot-Arm!\n");
  printf("---------------------------\n");
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
  printf("q to quit\n");
  printf("---------------------------\n");

//  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf J5: %.3lf J6: %.3lf\n",
//         joints_value_[0],
//         joints_value_[1],
//         joints_value_[2],
//         joints_value_[3],
//         joints_value_[4],
//         joints_value_[5]);
//  printf("       \n");
//  printf("---------------------------\n");

}

void XbotArmTeleop::setGoal(char ch)
{
  xarm_driver::SingleJointControl joint_control;
  if(ch == 'r' || ch == 'R')
  {
    printf("input : r \tincrease(++) joint 1 angle\n");
    joint_control.id = 1;
    joint_control.rad = joints_value_[0] + JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }
  else if(ch == 'f' || ch == 'F')
  {
    printf("input : f \tdecrease(++) joint 1 angle\n");
    joint_control.id = 1;
    joint_control.rad = joints_value_[0] - JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");

  }
  else if(ch == 't' || ch == 'T')
  {
    printf("input : t \tdincrease(--) joint 2 angle\n");
    joint_control.id = 2;
    joint_control.rad = joints_value_[1] + JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }

  else if(ch == 'g' || ch == 'G')
  {
    printf("input : G \tdecrease(++) joint 2 angle\n");
    joint_control.id = 2;
    joint_control.rad = joints_value_[1] - JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }
  else if(ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 3 angle\n");
    joint_control.id = 3;
    joint_control.rad = joints_value_[2] + JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }
  else if(ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 3 angle\n");
    joint_control.id = 3;
    joint_control.rad = joints_value_[2] - JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }

  else if(ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 4 angle\n");
    joint_control.id = 4;
    joint_control.rad = joints_value_[3] + JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }
  else if(ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 4 angle\n");
    joint_control.id = 4;
    joint_control.rad = joints_value_[3] - JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }

  else if(ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 5 angle\n");
    joint_control.id = 5;
    joint_control.rad = joints_value_[4] + JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }
  else if(ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 5 angle\n");
    joint_control.id = 5;
    joint_control.rad = joints_value_[4] - JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }

  else if(ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 6 angle\n");
    joint_control.id = 6;
    joint_control.rad = joints_value_[5] + JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }
  else if(ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 6 angle\n");
    joint_control.id = 6;
    joint_control.rad = joints_value_[5] - JOINT_DELTA;
    single_arm_pub_.publish(joint_control);
    ROS_INFO("Finish");
  }

  else if(ch == 'N' || ch == 'n')
  {
    printf("input : g \topen gripper\n");
    std_msgs::Bool msg;
    msg.data = false;
    grip_pub_.publish(msg);
    ROS_INFO("Gripper open");
  }
  else if(ch == 'm' || ch == 'M')
  {
    printf("input : f \tclose gripper\n");
    std_msgs::Bool msg;
    msg.data = true;
    grip_pub_.publish(msg);
    ROS_INFO("Gripper close");
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


  arm_teleop.printText();

  char ch;
  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    ros::spinOnce();
    arm_teleop.printText();
    ros::spinOnce();
    arm_teleop.setGoal(ch);
  }
  printf("input : q \tTeleop. is finished\n");
  arm_teleop.restoreTerminalSettings();

  return 0;
}
