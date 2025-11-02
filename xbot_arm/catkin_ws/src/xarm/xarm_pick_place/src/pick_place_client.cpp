#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include "xarm_pick_place/AddObject.h"
#include "xarm_pick_place/RemoveObjects.h"
#include "xarm_pick_place/TargetPickPose.h"
#include "xarm_pick_place/TargetPlacePose.h"
#include "xarm_pick_place/ArmControl.h"
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <math.h>
#include <algorithm>
#include <iterator>
#include <std_msgs/Empty.h>
#include <xarm_pick_place/ArmControl.h>
class PickPlaceClient{
public:
  PickPlaceClient(ros::NodeHandle nodehandle){
    control_nodehandle =nodehandle;
    add_object_client_ = control_nodehandle.serviceClient<xarm_pick_place::AddObject>("/arm/add_object");
    remove_object_client_ = control_nodehandle.serviceClient<xarm_pick_place::RemoveObjects>("/arm/remove_object");

    pick_client_ = control_nodehandle.serviceClient<xarm_pick_place::TargetPickPose>("/arm/pickup");
    place_client_ = control_nodehandle.serviceClient<xarm_pick_place::TargetPlacePose>("/arm/place");
    grip_pub_ = control_nodehandle.advertise<std_msgs::Bool>("arm/commands/grip",10);
    control_sub_ = control_nodehandle.subscribe(std::string("/arm/demo_pick_place"), 10,
                                                &PickPlaceClient::subscribeDisplay, this);
    arm_control_ = control_nodehandle.serviceClient<xarm_pick_place::ArmControl>("/arm/joints_control");

  }
  ~PickPlaceClient(){};

  ros::NodeHandle control_nodehandle;
  ros::Publisher grip_pub_;
  ros::ServiceClient pick_client_,arm_control_;
  ros::ServiceClient place_client_,add_object_client_,remove_object_client_ ;
  ros::Subscriber control_sub_;

  bool addDesk(){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "desk";
    geometry_msgs::Pose object_pose;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 1.2;
    primitive.dimensions[2] = 0.04;
    object_pose.orientation.w = 1.0;
    object_pose.position.x = 0.0;
    object_pose.position.y = 0;
    object_pose.position.z = -0.02;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;

    xarm_pick_place::AddObject add_object;
    add_object.request.collision_object = collision_object;

    if(add_object_client_.call(add_object)){
      if(add_object.response.success){
        ROS_INFO("success to add object!");
        return true;
      }else {
        ROS_ERROR("failed to call add object service!");
        return false;
      }
    }else
    {
        ROS_ERROR("Failed to call service add object");
        return false;
    }

  }

  bool callARMControl(){
    xarm_pick_place::ArmControl arm_control;
    arm_control.request.joints_values = {0, 0, 0, 0, 0, 0};
    if(arm_control_.call(arm_control)){
      if(arm_control.response.success){
        return true;
      }else {
        ROS_ERROR("Failed to call service arm/joints_control");

        return false;
      }
    }else {
      ROS_ERROR("Failed to call service arm/joints_control");

      return false;

    }
  }

  void subscribeDisplay(const std_msgs::Int16 &msg){
    ROS_INFO("Start the pick and place demo ...");
    int loop_times =msg.data;
    addDesk();
    if(msg.data == 0 )
      loop_times = 1;
    for (int i = 1;  i<=loop_times; i++) {
      demo();
    }

    sleep(1);
    removeObject("desk");
    removeObject("water_bottle");

  }

  void demo(){
    callAddBottleSrv(0.35-0.007,0.35-0.007,0.0,"bottle");

    // 请求抓取
    if(!callPickSrv(0.35, 0.35, 0.1)){
      ROS_ERROR("Failed to pick ");
      exit(1);
    }

    // 请求放置
    if(!callPlaceSrv(0.35, -0.35, 0.1)){
      ROS_ERROR("Failed to place");
      exit(1);
    }

    if(!callARMControl()){
      ROS_ERROR("Failed to reset");
      exit(1);
    }

    callAddBottleSrv(0.35-0.007,-0.35+0.009,0.0,"bottle");

    // 请求抓取
    if(!callPickSrv(0.35,-0.35, 0.1)){
      ROS_ERROR("Failed to pick ");
      exit(1);
    }

    // 请求放置
    if(!callPlaceSrv(0.35, 0.35, 0.1)){
      ROS_ERROR("Failed to place");
      exit(1);
    }

    if(!callARMControl()){
      ROS_ERROR("Failed to reset");
      exit(1);
    }

  }
  bool removeObject(std::string object_name){
    xarm_pick_place::RemoveObjects remove_object;
    remove_object.request.objects_id.push_back(object_name);
    if(remove_object_client_.call(remove_object)){
      if(remove_object.response.success){
        ROS_INFO("success to remove object!");
        sleep(1);
        return true;
      }else {
        ROS_ERROR("failed to call remove object service!");
        return false;
      }
    }else
    {
        ROS_ERROR("Failed to call service remove object");
        return false;
    }

  }
  bool callAddBottleSrv(double x, double y, double z, std::string object_name){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";

    /* The id of the object is used to identify it. */
    collision_object.id = object_name;
    // 此处的路径必须是ROS中可以找到的包路径
    shapes::Mesh* m = shapes::createMeshFromResource("package://xarm_pick_place/meshes/water_bottle_small.DAE");
    shape_msgs::Mesh shelf_mesh;
    shapes::ShapeMsg shelf_mesh_msg;
    shapes::constructMsgFromShape(m,shelf_mesh_msg);
    shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose shelf_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 3.1415926, 0);
    shelf_pose.orientation = tf2::toMsg(orientation);
    shelf_pose.position.x =  x;
    shelf_pose.position.y =  y;
    shelf_pose.position.z =  z + 0.01;

    collision_object.meshes.push_back(shelf_mesh);
    collision_object.mesh_poses.push_back(shelf_pose);
    collision_object.operation = collision_object.ADD;


//      std::vector<moveit_msgs::CollisionObject> target_objects;
//      collision_object.header.frame_id = "base_link";
//      geometry_msgs::Pose object_pose;

//      shape_msgs::SolidPrimitive primitive;

//      // 添加圆柱形的目标物体
//        primitive.type = primitive.CYLINDER;
//        primitive.dimensions.resize(2);
//        primitive.dimensions[0] = 0.12;
//        primitive.dimensions[1] = 0.03;
//        object_pose.orientation.w = 1.0;
//        object_pose.position.x = x;
//        object_pose.position.y = y;
//        object_pose.position.z = 0.06 + z;


//      collision_object.primitives.push_back(primitive);
//      collision_object.primitive_poses.push_back(object_pose);
//      collision_object.operation = collision_object.ADD;



    xarm_pick_place::AddObject add_object;
    add_object.request.collision_object = collision_object;

    if(add_object_client_.call(add_object)){
      if(add_object.response.success){
        ROS_INFO("success to add object!");
        sleep(1);
        return true;

      }else {
        ROS_ERROR("failed to call add object service!");
        return false;
      }
    }else
    {
        ROS_ERROR("Failed to call service add object");
        return false;
    }

  }


  bool callPickSrv(double x, double y, double z){
    xarm_pick_place::TargetPickPose pick_pose;
    pick_pose.request.x = x;
    pick_pose.request.y = y;
    pick_pose.request.z = z;
    if(pick_client_.call(pick_pose)){
      if(pick_pose.response.success){
        ROS_INFO("success to call pick service!");
        return true;
      }else {
        ROS_ERROR("failed to call pick service!");
        return false;
      }
    }else
    {
        ROS_ERROR("Failed to call service arm/pick_place/pick_pose");
        return false;
    }
  }

  bool callPlaceSrv(double x, double y, double z){
    xarm_pick_place::TargetPlacePose place_pose;
    place_pose.request.x = x;
    place_pose.request.y = y;
    place_pose.request.z = z;
    if(place_client_.call(place_pose)){
      if(place_pose.response.success){
        ROS_INFO("success to call place service!");
        return true;
      }else {
        ROS_ERROR("failed to call placeservice!");
        return false;
      }
    }else
    {
        ROS_ERROR("Failed to call service arm/pick_place/place_pose");
        return false;
    }


  }
private:

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_place_client");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(6);
  spinner.start();
  PickPlaceClient pick_place_client(nh);

  //sleep(5);
  ros::waitForShutdown();



  return 0;
}

