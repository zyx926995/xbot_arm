/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps
#include <ros/ros.h>
#include <iostream>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifndef PICK_PLACE_GENERATE_H_
#define PICK_PLACE_GENERATE_H_

namespace xarm{
static const double PAI = 3.1415926;

static const double UP_OBJECT_DISTANCE = 0.08;
static const double LEVEL_OBJECT_DISTANCE = 0.06;
static const double RETREAT_DISTANCE = 0.08;

struct PickPlaceData{
   int pre_pick_angle;
   int pre_place_angle;
};


// 给一个目标点和固定高度的水瓶，生成从上方落下，或者从其他地方过去抓取的可行路径。
// 这是仿真调试用的例子，运行此节点前需要先启动xarm_moveit_config/demo.launch
class PickAndPlace{
public:
  PickAndPlace(ros::NodeHandle nh, moveit::planning_interface::PlanningSceneInterface& planning_interface){
    nh_=nh;
    planning_scene_interface_ = planning_interface;
  };
  ~PickAndPlace(){};

  int pre_pick_angle_;
  int pre_place_angle_;
  int use_real_robot_;
  std::vector<double> pre_pick_joints_;
  std::vector<double> pre_place_joints_;


  /** @brief 添加桌面约束
   */
  void addDeskFloor();

  /** @brief 加载机械臂抓取和放置配置的相关参数
   *  @return true if load params succeeds
   */
  bool loadArmConfigData(std::string config_group);

  /** @brief 判断使用仿真进行轨迹的初步计算，还是用真实机器人进行展示
   *  @return true if use real arm
   */
  bool useRealArm();

  /** @brief 添加被抓取的物体
   *  @param planning_scene
   *  @return true if load params succeeds
   */
  void setTargetObject();

  /** @brief 计算抓取的距离：从上方向下抓取的话，pre_grasp_pose的高度要至少比物体高8cm
   */
  void setPreGraspDistance();

  /** @brief 计算可用于抓取操作的pre_pick_pose和轨迹
   *  @param arm_group:机械臂的movegroup
   *  @return true
   */
  bool computeUPTrajForPickup(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& gripper_group);

  /** @brief 计算放置点和轨迹
   *  @param arm_group:机械臂的movegroup
   *  @return true
   */
  bool computeTrajForPlace(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& gripper_group);

  /** @brief 抓到或者放下物体后，退出(默认向上移动5cm)
   *  @param arm_group:机械臂的movegroup
   *  @return true
   */
  bool retreatUP(double distance,moveit::planning_interface::MoveGroupInterface& arm_group);

  /** @brief 根据仿真计算出的可行轨迹，进行细致规划并控制机械臂进行抓取
   *  @param arm_group:机械臂的movegroup
   *  @param gripper_group:手爪的movegroup
   *  @return true
   */
  bool excuteAndPlanPickup(moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group);

  /** @brief 根据仿真计算出的可行轨迹，进行细致规划并控制机械臂进行放置
   *  @param arm_group:机械臂的movegroup
   *  @param gripper_group:手爪的movegroup
   *  @return true
   */
  bool excuteAndPlanPlace(moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group);


  /** @brief 张开手爪
   *  @param gripper_group:手爪的movegroup
   *  @return true
   */
  bool openGripper(moveit::planning_interface::MoveGroupInterface& gripper_group);

  /** @brief 闭合手爪
   *  @param gripper_group:手爪的movegroup
   *  @return true
   */
  bool closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group);


  void coutPreAngle();

private:

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;


  ros::NodeHandle nh_;
  std::string base_link_;
  std::string object_name_,object_type_;
  std::vector<double> object_size_,object_pose_;
  double object_grasp_height_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> pre_grasp_posture_,grasp_posture_;
  std::vector<double> pre_grasp_approach_,post_grasp_retreat_;
  std::vector<double> place_pose_,post_place_retreat_;

  geometry_msgs::Pose pick_pose_, pre_pick_pose_;
  geometry_msgs::Pose actual_place_pose_, pre_place_pose_;


  double pre_grasp_distance_;
  std::vector<std::string> object_ids_;


  int cyclicPickPlan(int init_i, int end_i, int gap, moveit::planning_interface::MoveGroupInterface& arm_group);
  double cyclicAllPickPlan(int init_i, int end_i, int gap, moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group);

  int cyclicPlacePlan(int init_i, int end_i, int gap, moveit::planning_interface::MoveGroupInterface& arm_group);
  double cyclicAllPlacePlan(int init_i, int end_i, int gap, moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& gripper_group);

};






} // namespace

#endif
