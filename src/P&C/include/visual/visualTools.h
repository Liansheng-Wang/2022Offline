#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class VisualTool
{
public:
  VisualTool(){
    initMarker();
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &VisualTool::PoseCb, this);
    odomMarkPub_ = nh_.advertise<visualization_msgs::Marker>("/visual/uav/pose", 1);
  }

private:
  void initMarker(){
    msgMarkerOdom_.action = visualization_msgs::Marker::ADD;
    msgMarkerOdom_.mesh_resource = std::string("package://controller/visual/meshes/uav.dae");
    msgMarkerOdom_.type = visualization_msgs::Marker::MESH_RESOURCE;
    msgMarkerOdom_.ns = "odom";
    msgMarkerOdom_.id = 0;
    msgMarkerOdom_.header.frame_id = "map";  
    msgMarkerOdom_.color.a = 0.9;
    msgMarkerOdom_.color.b = 153.0/255.0;
    msgMarkerOdom_.color.g = 204.0/255.0;
    msgMarkerOdom_.color.r = 102.0/255.0;
    msgMarkerOdom_.scale.x = 4;
    msgMarkerOdom_.scale.y = 4;
    msgMarkerOdom_.scale.z = 4;
  }

  void PoseCb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    msgMarkerOdom_.pose.position.x = msg->pose.pose.position.x;
    msgMarkerOdom_.pose.position.y = msg->pose.pose.position.y;
    msgMarkerOdom_.pose.position.z = msg->pose.pose.position.z;
    msgMarkerOdom_.pose.orientation.w = msg->pose.pose.orientation.w;
    msgMarkerOdom_.pose.orientation.x = msg->pose.pose.orientation.x;
    msgMarkerOdom_.pose.orientation.y = msg->pose.pose.orientation.y;
    msgMarkerOdom_.pose.orientation.z = msg->pose.pose.orientation.z;
    odomMarkPub_.publish(msgMarkerOdom_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odomSub_;
  ros::Publisher  odomMarkPub_;

  visualization_msgs::Marker msgMarkerOdom_;
};