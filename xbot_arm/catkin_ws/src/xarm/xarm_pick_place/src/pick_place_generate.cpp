#include <geometry_msgs/Pose.h>
#include "pick_place_generate.h"

namespace xarm {
bool PickAndPlace::loadArmConfigData(std::string config_group)
{
  // Load a param
  if (!nh_.hasParam("base_link"))
  {
    ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `base_link` missing from rosparam server. Did you load your end effector's configuration yaml file? Searching in namespace: " << nh_.getNamespace());
    return false;
  }
  nh_.getParam("base_link", base_link_);

  // Search within the sub-namespace of this end effector name
  ros::NodeHandle child_nh(nh_, config_group);

  // 加载真机demo相关的代码
  child_nh.getParam("use_real_robot", use_real_robot_);
  ROS_INFO_STREAM("simulation or real robot: "<<use_real_robot_);
  child_nh.getParam("pre_pick_angle", pre_pick_angle_);
  child_nh.getParam("pre_place_angle", pre_place_angle_);

  // Load a param
  if (!child_nh.hasParam("object_name"))
  {
    ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `object_name` missing from rosparam server. Did you load your end effector's configuration yaml file? Searching in namespace: " << child_nh.getNamespace());
    return false;
  }
  child_nh.getParam("object_name", object_name_);

  // Load a param
  if (!child_nh.hasParam("object_type"))
  {
    ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `object_type` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;
  }
  child_nh.getParam("object_type", object_type_);

  if(child_nh.hasParam("object_size"))
  {
    XmlRpc::XmlRpcValue object_size;
    child_nh.getParam("object_size", object_size);
    ROS_ASSERT(object_size.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < object_size.size(); ++i)
    {
      ROS_ASSERT(object_size[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      object_size_.push_back(static_cast<double>(object_size[i]));
    }
  }

  if(child_nh.hasParam("object_pose"))
  {
    XmlRpc::XmlRpcValue object_pose;
    child_nh.getParam("object_pose", object_pose);
    ROS_ASSERT(object_pose.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < object_pose.size(); ++i)
    {
      ROS_ASSERT(object_pose[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      object_pose_.push_back(static_cast<double>(object_pose[i]));
    }
  }

  child_nh.getParam("object_grasp_height", object_grasp_height_);

  XmlRpc::XmlRpcValue joint_list;
  child_nh.getParam("gripper_joints", joint_list);
  if (joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    for (int32_t i = 0; i < joint_list.size(); ++i)
    {
      ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      gripper_joints_.push_back(static_cast<std::string>(joint_list[i]));
      ROS_INFO_STREAM(gripper_joints_[i]);
    }
  else
    ROS_ERROR_STREAM_NAMED("temp","joint list type is not type array???");

  if(child_nh.hasParam("pregrasp_posture"))
  {
    XmlRpc::XmlRpcValue pregrasp_posture;
    child_nh.getParam("pregrasp_posture", pregrasp_posture);
    ROS_ASSERT(pregrasp_posture.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < pregrasp_posture.size(); ++i)
    {
      ROS_ASSERT(pregrasp_posture[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pre_grasp_posture_.push_back(static_cast<double>(pregrasp_posture[i]));
    }
  }


  if(child_nh.hasParam("grasp_posture"))
  {
    XmlRpc::XmlRpcValue grasp_posture;
    child_nh.getParam("grasp_posture", grasp_posture);
    ROS_ASSERT(grasp_posture.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < grasp_posture.size(); ++i)
    {
      ROS_ASSERT(grasp_posture[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      grasp_posture_.push_back(static_cast<double>(grasp_posture[i]));
    }
  }



  XmlRpc::XmlRpcValue pre_grasp_approach_list;
  child_nh.getParam("pre_grasp_approach", pre_grasp_approach_list);
  ROS_ASSERT(pre_grasp_approach_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < pre_grasp_approach_list.size(); ++i)
  {
    // Cast to double OR int
    if (pre_grasp_approach_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (pre_grasp_approach_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `pre_grasp_approach` wrong data type - int or double required.");
        return false;
      }
      else
        pre_grasp_approach_.push_back(static_cast<int>(pre_grasp_approach_list[i]));
    }
    else
      pre_grasp_approach_.push_back(static_cast<double>(pre_grasp_approach_list[i]));
  }


  XmlRpc::XmlRpcValue post_grasp_retreat;
  child_nh.getParam("post_grasp_retreat", post_grasp_retreat);
  ROS_ASSERT(post_grasp_retreat.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < post_grasp_retreat.size(); ++i)
  {
    // Cast to double OR int
    if (post_grasp_retreat[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (post_grasp_retreat[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `post_place_retreat` wrong data type - int or double required.");
        return false;
      }
      else
        post_grasp_retreat_.push_back(static_cast<int>(post_grasp_retreat[i]));
    }
    else
      post_grasp_retreat_.push_back(static_cast<double>(post_grasp_retreat[i]));
  }


  XmlRpc::XmlRpcValue place_pose;
  child_nh.getParam("place_pose", place_pose);
  ROS_ASSERT(place_pose.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < place_pose.size(); ++i)
  {
    // Cast to double OR int
    if (place_pose[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (place_pose[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `post_place_retreat` wrong data type - int or double required.");
        return false;
      }
      else
        place_pose_.push_back(static_cast<int>(place_pose[i]));
    }
    else
      place_pose_.push_back(static_cast<double>(place_pose[i]));
  }

  XmlRpc::XmlRpcValue post_place_retreat;
  child_nh.getParam("post_place_retreat", post_place_retreat);
  ROS_ASSERT(post_place_retreat.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < post_place_retreat.size(); ++i)
  {
    // Cast to double OR int
    if (post_place_retreat[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (post_place_retreat[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `post_place_retreat` wrong data type - int or double required.");
        return false;
      }
      else
        post_place_retreat_.push_back(static_cast<int>(post_place_retreat[i]));
    }
    else
      post_place_retreat_.push_back(static_cast<double>(post_place_retreat[i]));
  }

  XmlRpc::XmlRpcValue pre_pick_joints;
  child_nh.getParam("pre_pick_joints", pre_pick_joints);
  ROS_ASSERT(pre_pick_joints.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < pre_pick_joints.size(); ++i)
  {
    // Cast to double OR int
    if (pre_pick_joints[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (pre_pick_joints[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `pre_grasp_approach` wrong data type - int or double required.");
        return false;
      }
      else
        pre_pick_joints_.push_back(static_cast<int>(pre_pick_joints[i]));
    }
    else
      pre_pick_joints_.push_back(static_cast<double>(pre_pick_joints[i]));
  }


  XmlRpc::XmlRpcValue pre_place_joints;
  child_nh.getParam("pre_place_joints", pre_place_joints);
  ROS_ASSERT(pre_place_joints.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < pre_place_joints.size(); ++i)
  {
    // Cast to double OR int
    if (pre_place_joints[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (pre_place_joints[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `pre_grasp_approach` wrong data type - int or double required.");
        return false;
      }
      else
        pre_place_joints_.push_back(static_cast<int>(pre_place_joints[i]));
    }
    else
      pre_place_joints_.push_back(static_cast<double>(pre_place_joints[i]));
  }
  ROS_INFO("post_place_retreat");
  for(int i =0;i<post_place_retreat.size();i++){
    ROS_INFO_STREAM(post_grasp_retreat_[i]);
  }
  ROS_ERROR("111");

  ROS_INFO_STREAM( object_name_);

  return true;

}



void PickAndPlace::addDeskFloor(){
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = base_link_;
  collision_object.id = "desk_floor";
  geometry_msgs::Pose object_pose;

  shape_msgs::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 1.5;
  primitive.dimensions[2] = 0.04;
  object_pose.orientation.w = 1.0;
  object_pose.position.x = 0;
  object_pose.position.y = 0;
  object_pose.position.z = -0.02;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects_.push_back(collision_object);
}

bool PickAndPlace::useRealArm(){
  if(use_real_robot_ == 1)
    return true;
  else
    return false;
}

void PickAndPlace::setTargetObject(){
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = base_link_;
  collision_object.id = object_name_;
  geometry_msgs::Pose object_pose;

  shape_msgs::SolidPrimitive primitive;

  // 添加圆柱形的目标物体
  if (object_type_ == "CYLINDER"){
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = object_size_[0];
    primitive.dimensions[1] = object_size_[1];
    object_pose.orientation.w = 1.0;
    object_pose.position.x = object_pose_[0];
    object_pose.position.y = object_pose_[1];
    object_pose.position.z = object_pose_[2]+object_size_[0]/2.0 + 0.005;
   }

  // 添加长方体形的目标物体
  if (object_type_ == "BOX"){
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = object_size_[0];
    primitive.dimensions[1] = object_size_[1];
    primitive.dimensions[2] = object_size_[2];
    object_pose.orientation.w = 1.0;
    object_pose.position.x = object_pose_[0];
    object_pose.position.y = object_pose_[1];
    object_pose.position.z = object_pose_[2]+object_size_[3]/2.0;
   }

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects_.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(collision_objects_);

  object_ids_.push_back("water_bottle");

  pick_pose_.position.x = object_pose_[0];
  pick_pose_.position.y = object_pose_[1];
  pick_pose_.position.z = object_pose_[2] + object_grasp_height_;
}

void PickAndPlace::setPreGraspDistance(){
  // 沿着Z轴抓取
  if(pre_grasp_approach_[2]!=0){
    pre_pick_pose_.position.x = object_pose_[0];
    pre_pick_pose_.position.y = object_pose_[1];
    if (object_type_ == "CYLINDER"){
      pre_pick_pose_.position.z = object_pose_[2]+object_size_[0]+UP_OBJECT_DISTANCE;}
    if (object_type_ == "BOX"){
      pre_pick_pose_.position.z = object_pose_[2]+object_size_[2]+UP_OBJECT_DISTANCE;}
    pre_grasp_distance_ = pre_pick_pose_.position.z - pick_pose_.position.z;

  }
  ROS_INFO_STREAM("pre_grasp_distance_ = "<< pre_grasp_distance_);
  // TODO：添加水平抓取的计算

}

bool PickAndPlace::excuteAndPlanPickup(moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group){
  // 设置规划起点为当前位置
  arm_group.setStartStateToCurrentState();

  sensor_msgs::JointState joint_state;
  std::vector<std::string> joints_name = {"arm_1_joint", "arm_2_joint", "arm_3_joint",
                                          "arm_4_joint", "arm_5_joint", "arm_6_joint"};

  joint_state.header.stamp = ros::Time::now();
  joint_state.header.frame_id = "world";
  joint_state.name = joints_name;
  joint_state.position = pre_pick_joints_;
  arm_group.setJointValueTarget(joint_state);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
  arm_group.move();
  sleep(1);
  openGripper(gripper_group);
  sleep(2);

  arm_group.setStartStateToCurrentState();


  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);

  geometry_msgs::Pose target_pose1 = current_pose;

  target_pose1.position.z -= pre_grasp_distance_;
  waypoints.push_back(target_pose1);  // down

   moveit_msgs::RobotTrajectory trajectory1;
   const double jump_threshold = 0.0;
   const double eef_step = 0.01;
   double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
   ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
   if(abs(fraction - 1.0)<=0.15){
      arm_group.execute(trajectory1);
      closeGripper(gripper_group);
      sleep(2);
      geometry_msgs::PoseStamped pick_pose = arm_group.getCurrentPose();
      pick_pose_ = pick_pose.pose;
      // 抓取物体后，把物体关联到机械臂
      arm_group.attachObject(object_name_);
      return true;
   }
  return false;
}


bool PickAndPlace::excuteAndPlanPlace(moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group){
  // 根据抓取的实际位置，更新实际的放置高度
  actual_place_pose_.position.x = place_pose_[0];
  actual_place_pose_.position.y = place_pose_[1];
  actual_place_pose_.position.z = place_pose_[2] + pick_pose_.position.z;

  pre_place_pose_ = actual_place_pose_;
  pre_place_pose_.position.z += RETREAT_DISTANCE;


  // 设置规划起点为当前位置
  arm_group.setStartStateToCurrentState();

  sensor_msgs::JointState joint_state;
  std::vector<std::string> joints_name = {"arm_1_joint", "arm_2_joint", "arm_3_joint",
                                          "arm_4_joint", "arm_5_joint", "arm_6_joint"};

  joint_state.header.stamp = ros::Time::now();
  joint_state.header.frame_id = "world";
  joint_state.name = joints_name;
  joint_state.position = pre_place_joints_;
  arm_group.setJointValueTarget(joint_state);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
  }
  arm_group.move();
  arm_group.setStartStateToCurrentState();


  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);

  geometry_msgs::Pose target_pose1 = current_pose;

  target_pose1.position.z -= RETREAT_DISTANCE;
  waypoints.push_back(target_pose1);  // down

   moveit_msgs::RobotTrajectory trajectory1;
   const double jump_threshold = 0.0;
   const double eef_step = 0.01;
   double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
   ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
   if(abs(fraction - 1.0)<=0.15){
      arm_group.execute(trajectory1);
      openGripper(gripper_group);
      sleep(2);
      arm_group.detachObject(object_name_);
      return true;
   }
  return false;
}

bool PickAndPlace::openGripper(moveit::planning_interface::MoveGroupInterface& gripper_group){
  sensor_msgs::JointState joint_state;
  //std::vector<std::string> joints_name = {"gripper_1_joint","gripper_2_joint"};
  gripper_group.setStartStateToCurrentState();

  joint_state.header.stamp = ros::Time::now();
  joint_state.header.frame_id = "world";
  joint_state.name = gripper_joints_;
  joint_state.position = pre_grasp_posture_;
  gripper_group.setJointValueTarget(joint_state);
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//  bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//  if(!success){
//    ROS_ERROR("--- Plan trajectory failed --- ");
//  }
//  ROS_INFO("--- Plan trajectory Success and start moving --- ");
//  ROS_INFO("--- Open gripper! ---");

  gripper_group.move();
  return true;
}

bool PickAndPlace::closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group){
  sensor_msgs::JointState joint_state;

  joint_state.header.stamp = ros::Time::now();
  joint_state.header.frame_id = "world";
  joint_state.name = gripper_joints_;
  joint_state.position = grasp_posture_;
  gripper_group.setJointValueTarget(joint_state);
  gripper_group.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if(!success){
    ROS_ERROR("--- Plan trajectory failed --- ");
    return false;
  }
  gripper_group.move();
  ROS_INFO("--- Close gripper! ---");
  return true;
}

bool PickAndPlace::computeUPTrajForPickup(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& gripper_group){
  // 先删除添加的目标物体
  sleep(1);
  planning_scene_interface_.removeCollisionObjects(object_ids_);

  arm_group.setStartStateToCurrentState();

  // 如果目标点的x和y坐标都大于0，yaw计算的起始角度从0开始
  if( pre_pick_pose_.position.y>0){
    // 先调整步长大一点快速遍历
    int success_count = cyclicPickPlan(0,180,5,arm_group);
    ROS_ERROR_STREAM("i = "<<success_count);
    if (success_count != -1000){
      double rad = cyclicAllPickPlan(success_count + 4, success_count - 4,1,arm_group,gripper_group);
      ROS_ERROR_STREAM("rad success = "<< rad);
      if(rad != -1000){
        ROS_INFO_STREAM("--- Successfully planned the pickup path ---");
        geometry_msgs::PoseStamped pick_pose = arm_group.getCurrentPose();
        pick_pose_ = pick_pose.pose;
        return true;
      }
    }
  }else if(pre_pick_pose_.position.y<=0){
    int success_count = cyclicPickPlan(-180, 0, 5, arm_group);
    ROS_ERROR_STREAM("i = "<<success_count);
    if (success_count != -1000){
      double rad = cyclicAllPickPlan(success_count + 4, success_count - 4,1,arm_group, gripper_group);
      ROS_ERROR_STREAM("rad success = "<< rad);
      if(rad != -1000){
        ROS_INFO_STREAM("--- Successfully planned the pickup path ---");
        geometry_msgs::PoseStamped pick_pose = arm_group.getCurrentPose();
        pick_pose_ = pick_pose.pose;
        return true;
      }
    }
  }
  return false;
}

bool PickAndPlace::retreatUP(double distance,moveit::planning_interface::MoveGroupInterface& arm_group){
  ROS_INFO("start retreat");
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

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
   if(abs(fraction - 1.0)<=0.15){
      arm_group.execute(trajectory);
      ROS_INFO("retreat done!");
      return true;
   }
   return false;

}

bool PickAndPlace::computeTrajForPlace(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& gripper_group){

  // 根据抓取的实际位置，更新实际的放置高度
  actual_place_pose_.position.x = place_pose_[0];
  actual_place_pose_.position.y = place_pose_[1];
  actual_place_pose_.position.z = place_pose_[2] + pick_pose_.position.z;

  pre_place_pose_ = actual_place_pose_;
  pre_place_pose_.position.z += RETREAT_DISTANCE;

  // 如果目标点的x和y坐标都大于0，yaw计算的起始角度从0开始
  if( pre_place_pose_.position.y>0){
    // 先调整步长大一点快速遍历
    int success_count = cyclicPlacePlan(0,180,5,arm_group);
    ROS_ERROR_STREAM("i = "<<success_count);
    if (success_count != -1000){
      double rad = cyclicAllPlacePlan(success_count + 4, success_count - 4,1,arm_group, gripper_group);
      ROS_ERROR_STREAM("rad success = "<< rad);
      if(rad != -1000){
        ROS_INFO_STREAM("--- Successfully planned the pickup path ---");
        geometry_msgs::PoseStamped place_pose = arm_group.getCurrentPose();
        actual_place_pose_ = place_pose.pose;
        return true;
      }
    }
  }else if(pre_place_pose_.position.y<=0){
    int success_count = cyclicPlacePlan(-180, 0, 5, arm_group);
    ROS_ERROR_STREAM("i = "<<success_count);
    if (success_count != -1000){
      double rad = cyclicAllPlacePlan(success_count + 4, success_count - 4,1,arm_group, gripper_group);
      ROS_ERROR_STREAM("rad success = "<< rad);
      if(rad != -1000){
        ROS_INFO_STREAM("--- Successfully planned the pickup path ---");

        actual_place_pose_ = arm_group.getCurrentPose().pose;
        return true;
      }
    }
  }
  return false;
}

int PickAndPlace::cyclicPickPlan(int init_i, int end_i, int gap, moveit::planning_interface::MoveGroupInterface& arm_group){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  for(int i = init_i; i<=end_i; i += gap){
    tf2::Quaternion orientation;
    double rad = i/180.0*PAI;
    orientation.setRPY(PAI, 0, rad);
    pre_pick_pose_.orientation = tf2::toMsg(orientation);
    arm_group.setPoseTarget(pre_pick_pose_);
    bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    std::cout<<rad <<" i ="<<i<<std::endl;
    if(success){
      return i;
    }

    rad = -i/180.0*PAI;
    orientation.setRPY(PAI, 0, rad);
    pre_pick_pose_.orientation = tf2::toMsg(orientation);
    arm_group.setPoseTarget(pre_pick_pose_);
    success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    std::cout<<rad <<" i ="<<i<<std::endl;
    if(success){
      return -i;
    }

  }
  return -1000;

}


int PickAndPlace::cyclicPlacePlan(int init_i, int end_i, int gap, moveit::planning_interface::MoveGroupInterface& arm_group){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  for(int i = init_i; i<=end_i; i += gap){
    tf2::Quaternion orientation;
    double rad = i/180.0*PAI;
    orientation.setRPY(PAI, 0, rad);
    pre_place_pose_.orientation = tf2::toMsg(orientation);
    arm_group.setPoseTarget(pre_place_pose_);
    bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    std::cout<<rad <<" i ="<<i<<std::endl;
    if(success){
      return i;
    }

    rad = -i/180.0*PAI;
    orientation.setRPY(PAI, 0, rad);
    pre_place_pose_.orientation = tf2::toMsg(orientation);
    arm_group.setPoseTarget(pre_place_pose_);
    success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    std::cout<<rad <<" i ="<<i<<std::endl;
    if(success){
      return -i;
    }

  }
  return -1000;

}


double PickAndPlace::cyclicAllPickPlan(int init_i, int end_i, int gap, moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  for(int i = init_i; i>=end_i; i -= gap){
    tf2::Quaternion orientation;
    double rad = i/180.0*PAI;
    orientation.setRPY(PAI, 0, rad);
    pre_pick_pose_.orientation = tf2::toMsg(orientation);
    arm_group.setPoseTarget(pre_pick_pose_);
    bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    std::cout<<rad <<" i ="<<i<<std::endl;
    if(success){
      arm_group.move();
      pre_pick_joints_ = arm_group.getCurrentJointValues();
      // 在真实机器人进行展示的模式下，到达pre_pick_pose后,张开手爪
      if(use_real_robot_ == 1){
        openGripper(gripper_group);
        sleep(1);
      }
      gripper_group.setStartStateToCurrentState();
      arm_group.setStartStateToCurrentState();

      geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(current_pose);

      geometry_msgs::Pose target_pose1 = current_pose;

      target_pose1.position.z -= pre_grasp_distance_;
      waypoints.push_back(target_pose1);  // down

       moveit_msgs::RobotTrajectory trajectory1;
       const double jump_threshold = 0.0;
       const double eef_step = 0.01;
       double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
       ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
       if(abs(fraction - 1.0)<=0.08){
          arm_group.execute(trajectory1);
          pre_pick_angle_ = i;
          return rad;
       }else {
            continue;
       }

    }
  }
  return -1000;

}

double PickAndPlace::cyclicAllPlacePlan(int init_i, int end_i, int gap, moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  for(int i = init_i; i>=end_i; i -= gap){
    tf2::Quaternion orientation;
    double rad = i/180.0*PAI;
    orientation.setRPY(PAI, 0, rad);
    pre_place_pose_.orientation = tf2::toMsg(orientation);
    arm_group.setPoseTarget(pre_place_pose_);
    bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    std::cout<<rad <<" i ="<<i<<std::endl;
    if(success){
      arm_group.move();
      pre_place_joints_ = arm_group.getCurrentJointValues();

      geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(current_pose);

      geometry_msgs::Pose target_pose1 = current_pose;

      target_pose1.position.z -= RETREAT_DISTANCE;
      waypoints.push_back(target_pose1);  // down

       moveit_msgs::RobotTrajectory trajectory1;
       const double jump_threshold = 0.0;
       const double eef_step = 0.01;
       double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
       ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
       if(abs(fraction - 1.0)<=0.15){
          arm_group.execute(trajectory1);
          pre_place_angle_ = i;
          return rad;
       }else {
            continue;
       }

    }
  }
  return -1000;
}


void PickAndPlace::coutPreAngle(){
 ROS_INFO("----------- All tests have done! ----------------");
 ROS_INFO_STREAM("pre_pick_angle = " << pre_pick_angle_);
 ROS_INFO_STREAM("pre_place_angle = "<< pre_place_angle_);
 std::cout<<"--- Cout pre_pick_joints :  "<<std::endl;
 for(int i =0; i<pre_pick_joints_.size();i++){
   std::cout<<pre_pick_joints_[i]<<", ";
 }
 std::cout<<std::endl;

 std::cout<<"--- Cout pre_place_joints : "<<std::endl;
 for(int i = 0; i<pre_place_joints_.size();i++){
   std::cout<<pre_place_joints_[i]<<", ";
 }
 std::cout<<std::endl;




}


}//namespace




int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_pick_place");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(6);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm_group("xarm");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO("pick and place test!");

  xarm::PickAndPlace pick_place(nh,planning_scene_interface);
  // 加载设置的参数
  if(!pick_place.loadArmConfigData("xarm"))
    ROS_ERROR("Load params error!");

  // 添加桌面障碍物
  pick_place.addDeskFloor();

  // 添加目标物体并设置抓取的位置
  pick_place.setTargetObject();

  // 计算抓取的距离：从上方向下抓取的话，pre_grasp_pose的高度要至少比物体高8cm
  pick_place.setPreGraspDistance();

  if(pick_place.useRealArm()){
    if(!pick_place.excuteAndPlanPickup(arm_group,gripper_group)){
      ROS_ERROR("Can not get plan and excute to pick up !");
      exit(1);
    }
    if(!pick_place.retreatUP(0.05,arm_group)){
      ROS_ERROR("Can not retreat !");
      exit(1);
    }
    if(!pick_place.excuteAndPlanPlace(arm_group,gripper_group)){
      ROS_ERROR("Can not get plan and excute to place !");
      exit(1);
    }
    if(!pick_place.retreatUP(0.12,arm_group)){
      ROS_ERROR("Can not retreat !");
      exit(1);
    }
    ROS_INFO(" pickup and place done");
    pick_place.coutPreAngle();

  }else {
    // 计算能顺利抓取的角度和状态
    if(!pick_place.computeUPTrajForPickup(arm_group, gripper_group)){
      ROS_ERROR("Can not get plan to pick up !");
      exit(1);
    }
    pick_place.retreatUP(0.05,arm_group);
    if(!pick_place.computeTrajForPlace(arm_group, gripper_group)){
      ROS_ERROR("Can not get plan to pilace !");
      exit(1);
    }
    pick_place.retreatUP(0.12,arm_group);
    pick_place.coutPreAngle();
  }



  ros::waitForShutdown();
  return 0;
}
