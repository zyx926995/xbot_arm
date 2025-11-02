#ifndef SIMPLE_KEYBOARD_H_
#define SIMPLE_KEYBOARD_H_
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <termios.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#define NUM_OF_JOINT 8
#define POSE_DELTA 0.01
#define JOINT_DELTA 0.15
#define PATH_TIME 0.5
const static std::string ARM_PLANNING_GROUP = "xarm";
const static std::string GRIPPER_PLANNING_GROUP = "gripper";
static const std::vector<double> TARGET_ARM_JOINT_POSITIONS = {-0.768527317515, -0.208508687927, -0.193944850917, 1.72721782516, -2.48678521346, -0.388948172222};
static const std::vector<double> OPEN_GRIPPER_POSE = {0.65,0.65};
static const std::vector<double> CLOSE_GRIPPER_POSE = {0.0,0.0};
static const std::vector<double> HOME_POSITION = {0, 0, 0, 0, 0, 0};


class XbotArmTeleop
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  ros::Publisher grip_pub_;
  ros::Publisher single_arm_pub_;
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  sensor_msgs::JointState joint_state_;
  std::vector<std::string> joints_name_ = { "arm_1_joint", "arm_2_joint", "arm_3_joint",
                                           "arm_4_joint", "arm_5_joint", "arm_6_joint","gripper_1_joint","gripper_2_joint"};
  std::vector<double> joints_value_ ;
  geometry_msgs::Pose goal_pose;


  struct termios oldt_;

 public:

  XbotArmTeleop();
  ~XbotArmTeleop();

  void initClient();
  void initSubscriber();

  /** @brief 张开手爪
   *  @param gripper_group:手爪的movegroup
   *  @return true if success
   */
  bool openGripper( moveit::planning_interface::MoveGroupInterface & gripper_group);

  /** @brief 闭合手爪
   *  @param gripper_group:手爪的movegroup
   *  @return true if success
   */
  bool closeGripper( moveit::planning_interface::MoveGroupInterface & gripper_group);


  /** @brief 移动某个关节的位置
   *  @param arm_group:手臂的movegroup
   *  @param index:关节索引
   *  @param joint_position:关节的目标位置
   *  @return true if success
   */
  bool moveSingleArm(moveit::planning_interface::MoveGroupInterface & arm_group, int index, double joint_position);

  /** @brief 到达指定的joint位置
   *  @param arm_group:手臂的movegroup
   *  @param pose: 目标关节位置
   *  @return true if success
   */
  bool reachJointTarget(moveit::planning_interface::MoveGroupInterface & arm_group,std::vector<double> pose);

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  //void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();

  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);

  bool setToolControl(std::vector<double> joint_angle);

  void printText();
  void setGoal(char ch);

  void restoreTerminalSettings(void);
  void disableWaitingForEnter(void);

};

#endif
