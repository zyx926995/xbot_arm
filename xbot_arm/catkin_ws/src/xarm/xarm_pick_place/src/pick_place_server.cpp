/** xarm pick place demo **/

#include "pick_place_server.h"
#include <math.h>
#include <algorithm>
#include <iterator>
namespace xbot_arm {

PickPlaceApi::PickPlaceApi(ros::NodeHandle nh, moveit::planning_interface::PlanningSceneInterface& planning_interface){
  nh_=nh;
  planning_scene_interface_ = planning_interface;
  // 加载参数设置
  if(!loadArmConfigData("xarm")){
    ROS_ERROR("Load params error!");
    exit(-1);
  }
  // addDeskFloor();
  // 定义自动添加环境障碍物的服务
  add_object_srv_ = nh_.advertiseService(std::string("/arm/add_object"),&PickPlaceApi::addObjectCall,this);
  // 定义删除环境障碍物的服务
  remove_object_srv_ = nh_.advertiseService(std::string("/arm/remove_object"),&PickPlaceApi::removeObjectCall,this);

  // 定义自动抓取服务的服务端
  arm_pick_srv_ = nh_.advertiseService(std::string("/arm/pickup"),&PickPlaceApi::armPickCall,this);
  // 定义自动放置物体服务的服务端
  arm_place_srv_ = nh_.advertiseService(std::string("/arm/place"),&PickPlaceApi::armPlaceCall,this);

  arm_control_srv_ = nh_.advertiseService(std::string("/arm/joints_control"),&PickPlaceApi::armControlCall,this);
  //grip_pub_ = nh_.advertise<std_msgs::Bool>("/arm/commands/grip",10);
  grip_pub_ = nh_.advertise<xarm_driver::SingleJointControl>("/arm/commands/single_joint_control",10);

}



bool PickPlaceApi::addObjectCall(xarm_pick_place::AddObject::Request &req,xarm_pick_place::AddObject::Response &res){
  collision_objects_.push_back(req.collision_object);
  collision_objects_ids_.push_back(req.collision_object.id);
  planning_scene_interface_.addCollisionObjects(collision_objects_);
  planning_scene_interface_.applyCollisionObjects(collision_objects_);
  ROS_INFO_STREAM("--- Add object "<<req.collision_object.id<<" for plannning. ");
  res.success =true;
  return true;
}

bool PickPlaceApi::removeObjectCall(xarm_pick_place::RemoveObjects::Request &req,xarm_pick_place::RemoveObjects::Response &res){
  planning_scene_interface_.removeCollisionObjects(req.objects_id);
  collision_objects_.clear();
  res.success = true;
  ROS_INFO_STREAM("--- Remove objects for plannning. ");
  return true;

}


bool PickPlaceApi::armPickCall(xarm_pick_place::TargetPickPose::Request &req,xarm_pick_place::TargetPickPose::Response &res){
  moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper_group(GRIP_GROUP);

  pick_target_pose_.resize(3);
  pick_target_pose_[0] = req.x;
  pick_target_pose_[1] = req.y;
  pick_target_pose_[2] = req.z;

  ROS_INFO_STREAM("-- Target pick pose :[ " << pick_target_pose_[0]<<", "<<pick_target_pose_[1]<<", "<<pick_target_pose_[2]<<" ]");
  // 添加目标物体
  //setTargetObject(req.x,req.y, 0);
  std::vector<double> pre_pick_pose = pick_target_pose_;

  // 到达抓取前的位置,选择水平抓取还是竖直抓取由参数配置文件决定
  if(pre_grasp_approach_[0] == 1){
    ROS_INFO("Pick from up");
    pre_pick_pose[2] += pre_grasp_approach_[1];
  }else if (pre_grasp_approach_[0] == 0){
    ROS_INFO("Pick from level");
    double yaw = atan2(pick_target_pose_[1], pick_target_pose_[0]);
    pre_pick_pose[1] -= pre_grasp_approach_[1] * sin(yaw);
    pre_pick_pose[0] -= pre_grasp_approach_[1] * cos(yaw);
  }
  ROS_INFO_STREAM("-- Pre pick pose :[ " << pre_pick_pose[0]<<", "<<pre_pick_pose[1]<<", "<<pre_pick_pose[2]<<" ]");
  if(!reachPoseTarget(arm_group,pre_pick_pose)){
    ROS_ERROR("Can not reach the pre_pick_pose");
    res.success = false;
    return true;
  }

  // 到达抓取前的位置并张开手爪
  ROS_INFO("--- Reach the pre_pick_pose and open griper --- ");
  openGripper(gripper_group);

  //planning_scene_interface_.removeCollisionObjects(object_ids_);
  sleep(1);

//  if(!reachPoseTarget(arm_group,pick_target_pose_ )){
//    ROS_ERROR("Can not reach the target_pick_pose");
//    res.success = false;
//    return true;
//  }
  if(!moveCartesianPath(pick_target_pose_[0],pick_target_pose_[1],pick_target_pose_[2],arm_group)){
    ROS_ERROR("Can not reach the target_pick_pose");
    res.success = false;
    return true;
  }

  ROS_INFO("--- Reach the target_pick_pose and close griper --- ");
  usleep(500000);
  std::vector<std::string> bottle_id;
  bottle_id.push_back("bottle");
  planning_scene_interface_.removeCollisionObjects(bottle_id);
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  closeGripper(gripper_group);
  usleep(500000);
  setTargetObject(current_pose.position.x,current_pose.position.y,0);
  arm_group.attachObject(object_name_);
  usleep(500000);
  current_pose = arm_group.getCurrentPose().pose;
  // 抓取物体后向上撤退一段距离
  if(!moveCartesianPathUpDown(post_grasp_retreat_[1],arm_group)){
    ROS_ERROR("Can not retreat");
    res.success = false;
    return true;
  }

  res.success = true;
  ROS_INFO("--- Pick demo done ---");
  return true;
}

bool PickPlaceApi::armPlaceCall(xarm_pick_place::TargetPlacePose::Request &req,xarm_pick_place::TargetPlacePose::Response &res){
  moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper_group(GRIP_GROUP);

  place_target_pose_.resize(3);
  place_target_pose_[0] = req.x;
  place_target_pose_[1] = req.y;
  place_target_pose_[2] = req.z;
  ROS_INFO_STREAM("-- Target place pose :[ " << place_target_pose_[0]<<", "<<place_target_pose_[1]<<", "<<place_target_pose_[2]<<" ]");

  //place_pose_[2] = pick_target_pose_[2];

  // 到达放置的位置上方
  std::vector<double> pre_place_pose = place_target_pose_;
  pre_place_pose[2] += pre_place_approach_[1];
  if(!reachPoseTarget(arm_group,pre_place_pose)){
    ROS_ERROR("Can not reach the pre_place_pose");
    res.success = false;
    return true;
  }
    // 到达准确的放置位置
//  if(!reachPoseTarget(arm_group,place_target_pose_ )){
//    ROS_ERROR("Can not reach the target_place_pose");
//    res.success = false;
//    return true;
//  }
  if(!moveCartesianPath(place_target_pose_[0],place_target_pose_[1],place_target_pose_[2],arm_group)){
    ROS_ERROR("Can not reach the target_place_pose");
    res.success = false;
    return true;
  }
  sleep(1);
    // 张开手爪
  openGripper(gripper_group);
  arm_group.detachObject(object_name_);

  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  setTargetObject(current_pose.position.x,current_pose.position.y,0);
  sleep(1);
  // 放置完成后撤退
  std::vector<double> retreat_pose;
  retreat_pose.resize(3);
  retreat_pose[0] = req.x;
  retreat_pose[1] = req.y;
  retreat_pose[2] = req.z;

  if(post_place_retreat_[0] == 1){
    ROS_INFO("Retreat from up");
    retreat_pose[2] += post_place_retreat_[1];
  }else if (post_place_retreat_[0] == 0){
    ROS_INFO("Retreat  from level");
    double yaw = atan2(retreat_pose[1], retreat_pose[0]);
    retreat_pose[0] -= pre_grasp_approach_[1] * cos(yaw);
    retreat_pose[1] -= pre_grasp_approach_[1] * sin(yaw);
  }
  if(!reachPoseTarget(arm_group,retreat_pose )){
    ROS_ERROR("Can not retreat");
  }

  // 闭合手抓
   closeGripper(gripper_group);

  res.success = true;
  ROS_INFO("--- Place demo done ---");
  return true;
}


bool PickPlaceApi::moveCartesianPath(double x, double y, double z,moveit::planning_interface::MoveGroupInterface& arm_group){
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  arm_group.setStartStateToCurrentState();
  std::string group_name = arm_group.getName();
  tf2::Quaternion orientation;
  double yaw ;
  yaw = atan2(y,x);

  if(gripper_state_ == 1){
    orientation.setRPY(0, 3.1415926/2.0, yaw);
  }else if(gripper_state_ == 0){
    orientation.setRPY(0.0, -0.03, yaw);
  }else{
    ROS_ERROR("Wrong gripper_state_ param value!");
    exit(-1);
  }


  std::vector<geometry_msgs::Pose> waypoints;
  //planning_scene_interface_.addCollisionObjects(collision_objects_);

  waypoints.push_back(current_pose);
  geometry_msgs::Pose target_pose = current_pose;
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  waypoints.push_back(target_pose);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan RETREAT (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  if(fabs(fraction - 1.0)<=0.15){
      moveit::planning_interface::MoveGroupInterface::Plan c_plan;
      c_plan.trajectory_ = trajectory;
      arm_group.execute(c_plan);
      ROS_INFO("excute done!");
      return true;
   }
   return false;
}
bool PickPlaceApi::armControlCall(xarm_pick_place::ArmControl::Request &req,xarm_pick_place::ArmControl::Response &res){
  moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
  std::vector<double> joints_values;
  for (int i =0; i<6; i++){
    joints_values.push_back(req.joints_values[i]);
  }
  if(reachJointTarget(arm_group, joints_values)){
    res.success =true;
    return true;
  }else{
    res.success = false;
    return false;
  }
}

// 加载参数
bool PickPlaceApi::loadArmConfigData(std::string config_group)
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


  // Load a param
  if (!child_nh.hasParam("object_name"))
  {
    ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `object_name` missing from rosparam server. Did you load your end effector's configuration yaml file? Searching in namespace: " << child_nh.getNamespace());
    return false;
  }
  child_nh.getParam("object_name", object_name_);
  ROS_INFO_STREAM("Object name = " << object_name_);

  child_nh.getParam("gripper_state", gripper_state_);
  ROS_INFO_STREAM("gripper_state: "<< gripper_state_ );
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


  XmlRpc::XmlRpcValue pre_place_approach_list;
  child_nh.getParam("pre_place_approach", pre_place_approach_list);
  ROS_ASSERT(pre_place_approach_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < pre_place_approach_list.size(); ++i)
  {
    // Cast to double OR int
    if (pre_place_approach_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (pre_place_approach_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("pick_place_config_data_loader","Grasp configuration parameter `pre_grasp_approach` wrong data type - int or double required.");
        return false;
      }
      else
        pre_place_approach_.push_back(static_cast<int>(pre_place_approach_list[i]));
    }
    else
      pre_place_approach_.push_back(static_cast<double>(pre_place_approach_list[i]));
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

  return true;

}

void PickPlaceApi::addDeskFloor(){
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = BASE_LINK;
  collision_object.id = "desk_floor";
  geometry_msgs::Pose object_pose;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.5;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 0.04;
  object_pose.orientation.w = 1.0;
  object_pose.position.x = 0.0;
  object_pose.position.y = 0;
  object_pose.position.z = -0.02;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects_.push_back(collision_object);
  planning_scene_interface_.addCollisionObjects(collision_objects_);
  planning_scene_interface_.applyCollisionObjects(collision_objects_);
}

bool PickPlaceApi::openGripper( moveit::planning_interface::MoveGroupInterface & gripper_group){
  //ROS_ERROR("TEST 111");
  xarm_driver::SingleJointControl single_control;
  single_control.id = 0;
  single_control.rad = 0.65;
  grip_pub_.publish(single_control);
  usleep(200000);
  grip_pub_.publish(single_control);
  sleep(3);
  return true;
//  sensor_msgs::JointState joint_state;
//  gripper_group.setStartStateToCurrentState();

//  gripper_group.setJointValueTarget(pre_grasp_posture_);
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//  if(!success){
//    ROS_ERROR("--- Plan trajectory failed --- ");
//    return false;
//  }
//  ROS_INFO("--- Open gripper! ---");
//  gripper_group.move();
//  return true;
}

bool PickPlaceApi::closeGripper( moveit::planning_interface::MoveGroupInterface & gripper_group){
//  std_msgs::Bool msg;
//  msg.data = true;
//  grip_pub_.publish(msg);

  //sleep(3);
  xarm_driver::SingleJointControl single_control;
  single_control.id = 0;
  single_control.rad = grasp_posture_[0];
  grip_pub_.publish(single_control);
  usleep(200000);
  grip_pub_.publish(single_control);
  sleep(3);
//  sensor_msgs::JointState joint_state;
//  gripper_group.setStartStateToCurrentState();

//  gripper_group.setJointValueTarget(grasp_posture_);
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//  bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("xbot_arm/command/pose", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//  if(!success){
//    xarm_driver::SingleJointControl single_control;
//    single_control.id = 0;
//    single_control.rad = grasp_posture_[0];
//    grip_pub_.publish(single_control);
//    sleep(3);
//    ROS_ERROR("--- Plan trajectory failed --- ");
//    return false;
//  }
//  ROS_INFO("--- Close gripper! ---");
//  gripper_group.move();
//  sleep(1);
  return true;
}


bool PickPlaceApi::reachJointTarget(moveit::planning_interface::MoveGroupInterface & arm_group,std::vector<double> pose){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  arm_group.setStartStateToCurrentState();
  arm_group.setJointValueTarget(pose);
  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("pick place ", "Visualizing reachJointTarget (joint space goal) %s", success ? "" : "FAILED");
  if(success){
    arm_group.move();
    return true;
  }
  return false;
}

bool PickPlaceApi::moveCartesianPathUpDown(double distance,moveit::planning_interface::MoveGroupInterface& arm_group){
  ROS_INFO("start moveCartesianPathUpDown");
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  arm_group.setStartStateToCurrentState();
  std::vector<geometry_msgs::Pose> waypoints;
  //planning_scene_interface_.addCollisionObjects(collision_objects_);

  waypoints.push_back(current_pose);
  geometry_msgs::Pose target_pose = current_pose;
  target_pose.position.z += distance;
  waypoints.push_back(target_pose);
   moveit_msgs::RobotTrajectory trajectory;
   const double jump_threshold = 0.0;
   const double eef_step = 0.01;
   double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
   ROS_INFO_NAMED("tutorial", "Visualizing plan RETREAT (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
   if(fabs(fraction - 1.0)<=0.15){
      //arm_group.execute(trajectory);
      moveit::planning_interface::MoveGroupInterface::Plan c_plan;
      c_plan.trajectory_ = trajectory;
      arm_group.execute(c_plan);
      ROS_INFO("excute done!");
      return true;
   }
   return false;
}

bool PickPlaceApi::moveUpDown(double distance,moveit::planning_interface::MoveGroupInterface& arm_group){
  ROS_INFO("start moveCartesianPathUpDown");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  arm_group.setStartStateToCurrentState();
  geometry_msgs::Pose target_pose;
  target_pose = current_pose;
  target_pose.position.z = current_pose.position.z + distance;
  arm_group.setPoseTarget(target_pose);
  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("pick place ", "Visualizing moveUpDown (joint space goal) %s", success ? "" : "FAILED");
  if(success){
    arm_group.move();
    return true;
  }
  return false;
}
bool PickPlaceApi::reachPoseTarget(moveit::planning_interface::MoveGroupInterface& arm_group, std::vector<double> pose){
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::Pose current_pose = arm_group.getCurrentPose().pose;
  arm_group.setStartStateToCurrentState();
  geometry_msgs::Pose target_pose;
  target_pose.position.x = pose[0];
  target_pose.position.y = pose[1];
  target_pose.position.z = pose[2];
  std::string group_name = arm_group.getName();
  tf2::Quaternion orientation;
  double yaw ;
  yaw = atan2(pose[1],pose[0]);

  if(gripper_state_ == 1){
    orientation.setRPY(3.1415926/2.0, 0.0, yaw);
  }else if(gripper_state_ == 0){
    orientation.setRPY(0.0, -0.03, yaw);
  }else{
    ROS_ERROR("Wrong gripper_state_ param value!");
    exit(-1);
  }

  target_pose.orientation = tf2::toMsg(orientation);
  arm_group.setPoseTarget(target_pose);
  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("pick place ", "Visualizing reachPoseTarget (joint space goal) %s", success ? "" : "FAILED");
  if(success){
    arm_group.move();
    return true;
  }
  return false;
}

void PickPlaceApi::setTargetObject(double x, double y, double z){
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> target_objects;
  collision_object.header.frame_id = BASE_LINK;
  collision_object.id = object_name_;
  geometry_msgs::Pose object_pose;

  shape_msgs::SolidPrimitive primitive;

  // 添加圆柱形的目标物体
  if (object_type_ == "CYLINDER"){
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = object_size_[0] - 0.02;
    primitive.dimensions[1] = object_size_[1];
    object_pose.orientation.w = 1.0;
    object_pose.position.x = x;
    object_pose.position.y = y;
    object_pose.position.z = object_size_[0]/2.0 + z;
   }

  // 添加长方体形的目标物体
  if (object_type_ == "BOX"){
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = object_size_[0];
    primitive.dimensions[1] = object_size_[1];
    primitive.dimensions[2] = object_size_[2] - 0.01;
    object_pose.orientation.w = 1.0;
    object_pose.position.x = x;
    object_pose.position.y = y;
    object_pose.position.z = object_size_[2]/2.0 + z;
   }

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  target_objects.push_back(collision_object);
  planning_scene_interface_.addCollisionObjects(target_objects);
  planning_scene_interface_.applyCollisionObjects(target_objects);

  object_ids_.push_back(object_name_);

//  moveit_msgs::CollisionObject collision_object;
//  collision_object.header.frame_id = "base_link";

//  /* The id of the object is used to identify it. */
//  collision_object.id = "bottle";
//  // 此处的路径必须是ROS中可以找到的包路径
//  shapes::Mesh* m = shapes::createMeshFromResource("package://xarm_pick_place/meshes/water_bottle_small.DAE");
//  shape_msgs::Mesh shelf_mesh;
//  shapes::ShapeMsg shelf_mesh_msg;
//  shapes::constructMsgFromShape(m,shelf_mesh_msg);
//  shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);

//  /* A pose for the box (specified relative to frame_id) */
//  geometry_msgs::Pose shelf_pose;
//  tf2::Quaternion orientation;
//  orientation.setRPY(0, 3.1415926, 0);
//  shelf_pose.orientation = tf2::toMsg(orientation);
//  shelf_pose.position.x =  x;
//  shelf_pose.position.y =  y;
//  shelf_pose.position.z =  z - 0.02;
//  collision_object.meshes.push_back(shelf_mesh);
//  collision_object.mesh_poses.push_back(shelf_pose);
//  collision_object.operation = collision_object.ADD;
//  std::vector<moveit_msgs::CollisionObject> target_objects;
//  target_objects.push_back(collision_object);
//  planning_scene_interface_.addCollisionObjects(target_objects);
//  //planning_scene_interface_.applyCollisionObjects(target_objects);
}


} // name space

int main(int argc, char **argv){
  ros::init(argc, argv, "pick_place_server");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(6);
  spinner.start();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO("pick and place test!");

  xbot_arm::PickPlaceApi pick_place(nh,planning_scene_interface);
  ros::waitForShutdown();
  return 0;

}
