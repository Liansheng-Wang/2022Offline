#pragma once

#include <iostream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
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
    detectMarkPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visual/planner/target", 1);
    visulThread_ = new std::thread(std::bind(&VisualTool::visualThread, this));
  }

  ~VisualTool(){
    FLAG_running_ = false;
    visulThread_->join();
    delete visulThread_;
    std::cout<< "\033[32m ---> VisualTool    closed ^_^ ! \033[0m" << std::endl;
  }

  // wps -> waypoints, ps -> pose 初始化的时候作这种框的显示
  void setTargetMarker(std::vector<Eigen::Vector3d>& wps, std::vector<double>& ps){
    detectMarkers_.markers.clear();
    for(int i=0; i < wps.size(); i++){
      auto tf_q = tf::createQuaternionFromRPY(0, 0, ps[i]/180*M_PI);
      visualization_msgs::Marker marker;
      marker.mesh_resource = std::string("package://controller/visual/meshes/circle.dae");
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = "target";
      marker.id = i;
      marker.header.frame_id = "map";
      marker.pose.position.x = wps[i][0];
      marker.pose.position.y = wps[i][1];
      marker.pose.position.z = wps[i][2];
      marker.pose.orientation.w = tf_q.w();
      marker.pose.orientation.x = tf_q.x();
      marker.pose.orientation.y = tf_q.y();
      marker.pose.orientation.z = tf_q.z();
      marker.color.a = 0.5;
      marker.color.b = 0;
      marker.color.g = 0;
      marker.color.r = 255;
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      detectMarkers_.markers.push_back(marker);
    }
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

  void visualThread(){
    ros::Rate loopRate(2);
    while(ros::ok() && FLAG_running_){
      if(!detectMarkers_.markers.empty() && detectMarkPub_.getNumSubscribers()){
        detectMarkPub_.publish(detectMarkers_);
      }
      loopRate.sleep();
    }
  }

  // 相当于需要重新显示这个路径
  // 全局只规划一次的路径可以从这里来规划。但是全局的路径肯定也会调整的！
  // 重规划的路径也需要从这里来显示吗？
  // FIXME: 这个问题先不处理。
  void reCalPath(double& totalTime, Eigen::Matrix<double, 6, 3>& PathCoe_){
    msgPath_.poses.clear();
    double tt_time;
    Eigen::Matrix<double, 6, 3> PathCoe;
    {
      std::lock_guard<std::mutex> tempMut(pathInfoMut_);
      tt_time = totalTime;
      PathCoe = PathCoe_;
    }
    Eigen::Matrix<double, 1, 6> TCT_p;
    TCT_p[0] = 1;
    geometry_msgs::PoseStamped point;
    for(double t = 0; t < tt_time; t += 0.1)
    {
      for(int i=1; i<6; i++){
        TCT_p[i] = std::pow(t, i);
      }
      auto pt  = TCT_p * PathCoe;
      point.pose.position.x = pt[0];
      point.pose.position.y = pt[1];
      point.pose.position.z = pt[2];
      msgPath_.poses.push_back(point);
    }
    for(int j=1; j<6; j++){
      TCT_p[j] = std::pow(tt_time, j);
    }
    Eigen::Vector3d pt  = TCT_p * PathCoe;
    point.pose.position.x = pt[0];
    point.pose.position.y = pt[1];
    point.pose.position.z = pt[2];
    msgPath_.poses.push_back(point);
    FLAG_pathChange_ = false;
  }

  void PoseCb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    if(odomMarkPub_.getNumSubscribers()){
      msgMarkerOdom_.pose.position.x = msg->pose.pose.position.x;
      msgMarkerOdom_.pose.position.y = msg->pose.pose.position.y;
      msgMarkerOdom_.pose.position.z = msg->pose.pose.position.z;
      msgMarkerOdom_.pose.orientation.w = msg->pose.pose.orientation.w;
      msgMarkerOdom_.pose.orientation.x = msg->pose.pose.orientation.x;
      msgMarkerOdom_.pose.orientation.y = msg->pose.pose.orientation.y;
      msgMarkerOdom_.pose.orientation.z = msg->pose.pose.orientation.z;
      odomMarkPub_.publish(msgMarkerOdom_);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odomSub_;
  ros::Subscriber coefPathSub_;
  ros::Publisher  visualPathPub_;
  ros::Publisher  odomMarkPub_;
  ros::Publisher  detectMarkPub_;

  std::mutex pathInfoMut_;
  std::atomic_bool FLAG_running_;
  std::atomic_bool FLAG_pathChange_;
  std::thread* visulThread_ = nullptr;
  visualization_msgs::Marker msgMarkerOdom_;
  visualization_msgs::MarkerArray detectMarkers_;

  nav_msgs::Path msgPath_;
};