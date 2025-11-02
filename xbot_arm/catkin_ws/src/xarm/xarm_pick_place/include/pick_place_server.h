#ifndef PICK_PLACE_DEMO_H_
#define PICK_PLACE_DEMO_H_
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "xarm_pick_place/AddObject.h"
#include "xarm_pick_place/RemoveObjects.h"
#include "xarm_pick_place/TargetPickPose.h"
#include "xarm_pick_place/TargetPlacePose.h"
#include "xarm_pick_place/ArmControl.h"
#include "xarm_driver/SingleJointControl.h"
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
const static double PAI = 3.1415926;
const static std::string ARM_GROUP = "xarm";
const static std::string GRIP_GROUP = "gripper";
const static std::string BASE_LINK = "base_link";

static const std::vector<double> PLACE_JOINT_POSITIONS ={-1.51480976759, -1.03185291482, -0.0710701490616, -1.45846900153, 0.054608905524, -0.679993637799};
namespace xbot_arm {

class PickPlaceApi
{
public:
  PickPlaceApi(ros::NodeHandle nh, moveit::planning_interface::PlanningSceneInterface& planning_interface);
  ~PickPlaceApi(){
    planning_scene_interface_.removeCollisionObjects(object_ids_);
  }
  /** @brief 加载机械臂抓取和放置配置的相关参数
   *  @return true if load params succeeds
   */
  bool loadArmConfigData(std::string config_group);

  /** @brief 添加障碍物信息的服务端
   *  @return true if add object success
   */
  bool addObjectCall(xarm_pick_place::AddObject::Request &req,xarm_pick_place::AddObject::Response &res);

  /** @brief 删除障碍物信息的服务端
   *  @return true if remove object success
   */
  bool removeObjectCall(xarm_pick_place::RemoveObjects::Request &req,xarm_pick_place::RemoveObjects::Response &res);

  /** @brief 桌面机械臂的pick和place服务端
   *  @return true if load params succeeds
   */
  bool armPickCall(xarm_pick_place::TargetPickPose::Request &req,xarm_pick_place::TargetPickPose::Response &res);
  bool armPlaceCall(xarm_pick_place::TargetPlacePose::Request &req,xarm_pick_place::TargetPlacePose::Response &res);


  /** @brief 桌面机械臂的关节控制
   *  @return true if load params succeeds
   */
  bool armControlCall(xarm_pick_place::ArmControl::Request &req,xarm_pick_place::ArmControl::Response &res);


  /** @brief 添加桌面约束
   */
  void addDeskFloor();
  /** @brief 添加目标物体
   */
  void setTargetObject(double x, double y, double z);

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

  /** @brief 到达指定的joint位置
   *  @param arm_group:手臂的movegroup
   *  @param pose: 目标关节位置
   *  @return true if success
   */
  bool reachJointTarget(moveit::planning_interface::MoveGroupInterface & arm_group,std::vector<double> pose);

  /** @brief 到达指定的pose位置，根据水平或竖直抓取，自动设定水平或竖直姿态，且只支持这两周姿态
   *  @param arm_group:手臂的movegroup
   *  @param pose: 目标关节位置
   *  @return true if success
   */
  bool reachPoseTarget(moveit::planning_interface::MoveGroupInterface& arm_group,std::vector<double> pose);


  /** @brief 笛卡尔坐标系内上下移动某段距离
   *  @param arm_group:手臂的movegroup
   *  @param distance: 移动的距离，带正负
   *  @return true if success
   */
  bool moveCartesianPathUpDown(double distance,moveit::planning_interface::MoveGroupInterface& arm_group);

  /** @brief 上下移动某段距离
   *  @param arm_group:手臂的movegroup
   *  @param distance: 移动的距离，带正负
   *  @return true if success
   */
  bool moveUpDown(double distance,moveit::planning_interface::MoveGroupInterface& arm_group);

  /** @brief 笛卡尔坐标系内移动某段距离
   *  @param arm_group:手臂的movegroup
   *  @param distance: 移动的距离，带正负
   *  @return true if success
   */
  bool moveCartesianPath(double x, double y, double z,moveit::planning_interface::MoveGroupInterface& arm_group);


private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
  ros::NodeHandle nh_;
  ros::ServiceServer add_object_srv_, remove_object_srv_, arm_pick_srv_;
  ros::ServiceServer arm_control_srv_, arm_place_srv_;
  std::vector<double> pick_target_pose_,place_target_pose_;
  ros::Publisher grip_pub_;


  std::vector<std::string> object_ids_, collision_objects_ids_;

  std::string base_link_;
  int gripper_state_;
  std::string object_name_,object_type_;
  std::vector<double> object_size_,object_pose_;
  double object_grasp_height_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> pre_grasp_posture_,grasp_posture_;
  std::vector<double> pre_grasp_approach_,post_grasp_retreat_;
  std::vector<double> place_pose_,pre_place_approach_,post_place_retreat_;

};

}

#endif
