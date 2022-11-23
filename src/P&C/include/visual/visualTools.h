#pragma once

#include <common/base.h>
#include <queue>
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
  VisualTool():FLAG_running_(true), FLAG_initDetect_(false),
    FLAG_globalChange_(false), FLAG_localChange_(false)
  {
    initMsg();
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &VisualTool::PoseCb, this);
    odomMarkPub_   = nh_.advertise<visualization_msgs::Marker>("/visual/uav/pose", 1);
    wpsMarkPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visual/planner/wps", 1);
    globalTrjPub_  = nh_.advertise<nav_msgs::Path>("/visual/planner/globaltrj", 1);
    localTrjPub_   = nh_.advertise<nav_msgs::Path>("/visual/planner/localtrj", 1);

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
      marker.color.r = 1;
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      detectMarkers_.markers.push_back(marker);
    }
    FLAG_initDetect_ = true;
  }

  void setTargetMarker(std::vector<Eigen::Vector3d>& wps, std::vector<double>& ps, std::vector<int> types){
    detectMarkers_.markers.clear();
    for(int i=0; i < wps.size(); i++){
      if(types[i] == 4){
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "target";
        marker.id = i;
        marker.header.frame_id = "map";
        marker.pose.position.x = wps[i][0];
        marker.pose.position.y = wps[i][1];
        marker.pose.position.z = wps[i][2];
        marker.color.a = 0.8;
        marker.color.b = 0;
        marker.color.g = 1;
        marker.color.r = 0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        detectMarkers_.markers.push_back(marker);
      }else{
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
        marker.color.a = 0.8;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 1;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        detectMarkers_.markers.push_back(marker);
      }
    }
    FLAG_initDetect_ = true;
  }

  // 数据处理在 VisualTool 的线程中慢慢做
  void setGlobalTrj(const PolynomialTraj& global_traj){
    globalTraj_ = global_traj;
    FLAG_globalChange_ = true;
  }

  void setLocalTrj(const PolynomialTraj& loca_traj){
    localTraj_ = loca_traj;
    FLAG_localChange_ = true;
  }

  // void setGlobalTrj(double& totalTime, Eigen::Matrix<double, 6, 3>& PathCoe_){
  //   nav_msgs::Path msgPath;
  //   msgPath.header.frame_id = "map";
  //   msgPath.header.stamp = ros::Time::now();
  //   double tt_time;
  //   Eigen::Matrix<double, 6, 3> PathCoe;
  //   {
  //     std::lock_guard<std::mutex> tempMut(pathInfoMut_);
  //     tt_time = totalTime;
  //     PathCoe = PathCoe_;
  //   }
  //   Eigen::Matrix<double, 1, 6> TCT_p;
  //   geometry_msgs::PoseStamped point;
  //   for(double t = 0; t < tt_time; t += 0.1)
  //   {
  //     for(int i=0; i<6; i++){
  //       TCT_p[i] = std::pow(t, i);
  //     }
  //     auto pt  = TCT_p * PathCoe;
  //     point.pose.position.x = pt[0];
  //     point.pose.position.y = pt[1];
  //     point.pose.position.z = pt[2];
  //     msgPath.poses.push_back(point);
  //   }
  //   for(int j=1; j<6; j++){
  //     TCT_p[j] = std::pow(tt_time, j);
  //   }
  //   Eigen::Vector3d pt  = TCT_p * PathCoe;
  //   point.pose.position.x = pt[0];
  //   point.pose.position.y = pt[1];
  //   point.pose.position.z = pt[2];
  //   msgPath.poses.push_back(point);
  //   msgGlobalPath_ = msgPath;
  //   FLAG_globalChange_ = true;
  // }

private:
  void initMsg(){
    msgMarkerOdom_.action = visualization_msgs::Marker::ADD;
    msgMarkerOdom_.mesh_resource = std::string("package://controller/visual/meshes/uav.dae");
    msgMarkerOdom_.type = visualization_msgs::Marker::MESH_RESOURCE;
    msgMarkerOdom_.ns = "odom";
    msgMarkerOdom_.id = 0;
    msgMarkerOdom_.header.frame_id = "map";  
    msgMarkerOdom_.color.a = 0.9;
    msgMarkerOdom_.color.b = 225/255.0;
    msgMarkerOdom_.color.g = 105/255.0;
    msgMarkerOdom_.color.r = 65/255.0;
    msgMarkerOdom_.scale.x = 4;
    msgMarkerOdom_.scale.y = 4;
    msgMarkerOdom_.scale.z = 4;

    msgGlobalPath_.header.frame_id = "map";
    msgGlobalPath_.header.stamp = ros::Time::now();
    msgLocalPath_.header.frame_id = "map";
    msgLocalPath_.header.stamp = ros::Time::now();
  }

  void visualThread(){
    ros::Rate loopRate(2);
    while(ros::ok() && FLAG_running_){
      if(FLAG_initDetect_){
        if(!detectMarkers_.markers.empty() && wpsMarkPub_.getNumSubscribers()){
          wpsMarkPub_.publish(detectMarkers_);
        }
      }

      if(FLAG_globalChange_){
        caluGlobalPath();
        FLAG_globalChange_ = false;
      }
      if(FLAG_localChange_){
        caluLocalPath();
        FLAG_localChange_ = false;
      }

      if(globalTrjPub_.getNumSubscribers() && !msgGlobalPath_.poses.empty()){
        globalTrjPub_.publish(msgGlobalPath_);
      }
      if(localTrjPub_.getNumSubscribers() && !msgLocalPath_.poses.empty()){
        localTrjPub_.publish(msgLocalPath_);
      }
      loopRate.sleep();
    }
  }

  void caluGlobalPath(){
    nav_msgs::Path msgPath;
    msgPath.header.frame_id = "map";
    msgPath.header.stamp = ros::Time::now();
    Eigen::Matrix<double, 1, 6> TCT_p;
    geometry_msgs::PoseStamped point;
    for(int i = 0; i < globalTraj_.times_.size(); i++){
      for(double t = 0; t <= globalTraj_.times_[i]; t += 0.1)
      {
        for(int j=0; j<6; j++){
          TCT_p[j] = std::pow(t, j);
        }
        auto pt  = TCT_p * globalTraj_.coefs_[i];
        point.pose.position.x = pt[0];
        point.pose.position.y = pt[1];
        point.pose.position.z = pt[2];
        msgPath.poses.push_back(point);
      }
    }
    msgGlobalPath_ = msgPath;
  }

  void caluLocalPath(){
    nav_msgs::Path msgPath;
    msgPath.header.frame_id = "map";
    msgPath.header.stamp = ros::Time::now();
    Eigen::Matrix<double, 1, 6> TCT_p;
    geometry_msgs::PoseStamped point;
    for(int i = 0; i < localTraj_.times_.size(); i++){
      for(double t = 0; t <= localTraj_.times_[i]; t += 0.1)
      {
        for(int j=0; j<6; j++){
          TCT_p[j] = std::pow(t, j);
        }
        auto pt  = TCT_p * localTraj_.coefs_[i];
        point.pose.position.x = pt[0];
        point.pose.position.y = pt[1];
        point.pose.position.z = pt[2];
        msgPath.poses.push_back(point);
      }
    }
    msgLocalPath_ = msgPath;
  }

  void PoseCb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    if(odomMarkPub_.getNumSubscribers() > 0){
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
  ros::Publisher  visualPathPub_;
  ros::Publisher  odomMarkPub_;
  ros::Publisher  wpsMarkPub_;
  ros::Publisher  globalTrjPub_;
  ros::Publisher  localTrjPub_;

  std::mutex pathInfoMut_;
  std::atomic_bool FLAG_running_;
  std::atomic_bool FLAG_globalChange_;
  std::atomic_bool FLAG_localChange_;
  std::atomic_bool FLAG_initDetect_;
  std::thread* visulThread_ = nullptr;
  visualization_msgs::Marker msgMarkerOdom_;
  visualization_msgs::MarkerArray detectMarkers_;
  nav_msgs::Path msgGlobalPath_, msgLocalPath_;

  PolynomialTraj globalTraj_;
  PolynomialTraj localTraj_; 

};