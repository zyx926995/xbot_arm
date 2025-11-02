/** xbot-arm hello demo **/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "geometry_msgs/Pose.h"
#include "xarm_driver/SingleJointControl.h"

const static std::string ARM_PLANNING_GROUP = "xarm";
const static std::string GRIPPER_PLANNING_GROUP = "gripper";
static const std::vector<double> TARGET_ARM_JOINT_POSITIONS = {0.768527317515, -0.208508687927, 0.193944850917, 1.72721782516, 2.48678521346, -0.388948172222};
static const std::vector<double> OPEN_GRIPPER_POSE = {0.65,0.65};
static const std::vector<double> CLOSE_GRIPPER_POSE = {0.0,0.0};
static const std::vector<double> HOME_POSITION = {0, 0, 0, 0, 0, 0};

class GreetDemo{
public:
  GreetDemo(ros::NodeHandle nh){
    nh_ = nh;
  }
  ~ GreetDemo(){};

  /** @brief 到达指定的pose位置
   *  @param arm_group:手臂的movegroup
   *  @param pose: 目标关节位置
   *  @return true if success
   */
  bool reachPoseTarget(moveit::planning_interface::MoveGroupInterface& arm_group, std::vector<double> pose){
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
    arm_group.setStartStateToCurrentState();
    geometry_msgs::Pose target_pose;
    target_pose.position.x = pose[0];
    target_pose.position.y = pose[1];
    target_pose.position.z = pose[2];
    target_pose.orientation.w =1;
    arm_group.setPoseTarget(target_pose);
    bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("pick place ", "Visualizing reachPoseTarget (joint space goal) %s", success ? "" : "FAILED");
    if(success){
      arm_group.move();
      return true;
    }
    return false;

  }

  /** @brief 到达指定的joint位置
   *  @param arm_group:手臂的movegroup
   *  @param pose: 目标关节位置
   *  @return true if success
   */
  bool reachJointTarget(moveit::planning_interface::MoveGroupInterface & arm_group,std::vector<double> pose){
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

  /** @brief 张开手爪
   *  @param gripper_group:手爪的movegroup
   *  @return true if success
   */
  bool openGripper( moveit::planning_interface::MoveGroupInterface & gripper_group){
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

  /** @brief 闭合手爪
   *  @param gripper_group:手爪的movegroup
   *  @return true if success
   */
  bool closeGripper( moveit::planning_interface::MoveGroupInterface & gripper_group){
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

  /** @brief 移动某个关节的位置
   *  @param arm_group:手臂的movegroup
   *  @param index:关节索引
   *  @param joint_position:关节的目标位置
   *  @return true if success
   */
  bool moveSingleArm(moveit::planning_interface::MoveGroupInterface & arm_group, int index, double joint_position){
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

  bool moveCartesianPathUpDown(double distance,moveit::planning_interface::MoveGroupInterface& arm_group){
    ROS_INFO("start moveCartesianPathUpDown");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
    arm_group.setStartStateToCurrentState();
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose);
    geometry_msgs::Pose target_pose = current_pose;
    target_pose.position.z += distance;
    waypoints.push_back(target_pose);  // UP
     moveit_msgs::RobotTrajectory trajectory;
     const double jump_threshold = 0.0;
     const double eef_step = 0.01;
     double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
     ROS_INFO_NAMED("tutorial", "Visualizing plan RETREAT (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
     if(fabs(fraction - 1.0)<=0.2){
       moveit::planning_interface::MoveGroupInterface::Plan my_plan;
       //my_plan.planning_time_ = ros::Time::now();
       //my_plan.start_state_ = arm_group.getCurrentState();
       my_plan.trajectory_ = trajectory;
       arm_group.execute(my_plan);
       ROS_INFO("retreat done!");
       return true;
     }
     return false;
  }
private:
  ros::NodeHandle nh_;


};


int main(int argc, char **argv){
  ros::init(argc, argv, "greet_demo", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  GreetDemo greet_demo(nh);
  ros::Publisher single_arm_pub_ = nh.advertise<xarm_driver::SingleJointControl>("arm/commands/single_joint_control",10);

  // 连接机械臂实例组
  moveit::planning_interface::MoveGroupInterface arm_group("xarm");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

  std::vector<double> pose = {0.35,0,0.4};
  if(!greet_demo.reachPoseTarget(arm_group,pose)){
    ROS_ERROR("Unable to reach the FirstGoal ");
    exit(1);
  }

  std::cout<<"--- Cout target joints :  "<<std::endl;
  for(int i =0; i<TARGET_ARM_JOINT_POSITIONS.size();i++){
    std::cout<<TARGET_ARM_JOINT_POSITIONS[i]<<", ";
  }
  std::cout<<std::endl;
  geometry_msgs::Pose real_pose = arm_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Pose :"<< real_pose.position.x << ","<< real_pose.position.y << ","<<real_pose.position.z);

  if(!greet_demo.openGripper(gripper_group)){
    ROS_ERROR("Unable to open hand ");
    exit(1);
  }
  sleep(1);
  if(!greet_demo.moveCartesianPathUpDown(+0.1,arm_group)){
    ROS_ERROR("Unable to move cartesian path Up ");
    exit(1);
  }
  if(!greet_demo.moveCartesianPathUpDown(-0.1,arm_group)){
    ROS_ERROR("Unable to move cartesian path down ");
    exit(1);
  }

  std::vector<double> joints = {0.0,0,1.57,-0.4,1.57,0};

  if(!greet_demo.reachJointTarget(arm_group,joints)){
    ROS_ERROR("Unable to reach the FirstGoal ");
    exit(1);
  }

  xarm_driver::SingleJointControl joint_control;
  joint_control.id = 4;
  joint_control.rad = 0.4;
  single_arm_pub_.publish(joint_control);
  sleep(2);
  joint_control.rad = -0.4;
  single_arm_pub_.publish(joint_control);
  sleep(2);
  joint_control.rad = 0.4;
  single_arm_pub_.publish(joint_control);
  sleep(3);
  if(!greet_demo.reachJointTarget(arm_group,HOME_POSITION)){
    ROS_ERROR("Unable to Return home  ");
    exit(1);
  }
  if(!greet_demo.closeGripper(gripper_group)){
    ROS_ERROR("Unable to close hand ");
    exit(1);
  }

  ROS_INFO_STREAM("CTRL C EXIT");
  ros::waitForShutdown();

}
