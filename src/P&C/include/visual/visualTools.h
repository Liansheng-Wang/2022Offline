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
    detectMarkPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visual/planner/target", 1);
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
      marker.color.r = 255;
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      detectMarkers_.markers.push_back(marker);
    }
    FLAG_initDetect_ = true;
  }


private:
  void initMsg(){
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

    // 初始化的时候先丢一个垃圾进去
    nav_msgs::Path msgPath;
    msgPath.header.frame_id = "map";
    msgPath.header.stamp = ros::Time::now();
    msgGlobalPath_.push(msgPath);
    msgLocalPath_.push(msgPath);
  }

  void visualThread(){
    ros::Rate loopRate(2);
    while(ros::ok() && FLAG_running_){
      if(FLAG_initDetect_){
        if(!detectMarkers_.markers.empty() && detectMarkPub_.getNumSubscribers()){
          detectMarkPub_.publish(detectMarkers_);
        }
      }
      if(FLAG_globalChange_){
        msgGlobalPath_.pop();
        FLAG_globalChange_ = false;
      }
      if(FLAG_localChange_){
        msgLocalPath_.pop();
        FLAG_localChange_ = false;
      }
      if(globalTrjPub_.getNumSubscribers() && !msgGlobalPath_.front().poses.empty()){
        globalTrjPub_.publish(msgGlobalPath_.front());
      }
      if(localTrjPub_.getNumSubscribers() && !msgLocalPath_.front().poses.empty()){
        localTrjPub_.publish(msgLocalPath_.front());
      }
      loopRate.sleep();
    }
  }

  // 相当于需要重新显示这个路径
  // 全局只规划一次的路径可以从这里来规划。但是全局的路径肯定也会调整的！
  // 重规划的路径也需要从这里来显示吗？
  void getGlobalTrj(double& totalTime, Eigen::Matrix<double, 6, 3>& PathCoe_){
    nav_msgs::Path msgPath;
    msgPath.header.frame_id = "map";
    msgPath.header.stamp = ros::Time::now();
    double tt_time;
    Eigen::Matrix<double, 6, 3> PathCoe;
    {
      std::lock_guard<std::mutex> tempMut(pathInfoMut_);
      tt_time = totalTime;
      PathCoe = PathCoe_;
    }
    Eigen::Matrix<double, 1, 6> TCT_p;
    geometry_msgs::PoseStamped point;
    for(double t = 0; t < tt_time; t += 0.1)
    {
      for(int i=0; i<6; i++){
        TCT_p[i] = std::pow(t, i);
      }
      auto pt  = TCT_p * PathCoe;
      point.pose.position.x = pt[0];
      point.pose.position.y = pt[1];
      point.pose.position.z = pt[2];
      msgPath.poses.push_back(point);
    }
    for(int j=1; j<6; j++){
      TCT_p[j] = std::pow(tt_time, j);
    }
    Eigen::Vector3d pt  = TCT_p * PathCoe;
    point.pose.position.x = pt[0];
    point.pose.position.y = pt[1];
    point.pose.position.z = pt[2];
    msgPath.poses.push_back(point);
    msgGlobalPath_.push(msgPath);
    FLAG_globalChange_ = true;
  }

  // 参数是直接拷贝进来的，应该还挺快的。
  void getGlobalTrj(PolynomialTraj global_traj){
    nav_msgs::Path msgPath;
    msgPath.header.frame_id = "map";
    msgPath.header.stamp = ros::Time::now();
    Eigen::Matrix<double, 1, 6> TCT_p;
    geometry_msgs::PoseStamped point;
    for(int i = 0; i < global_traj.times.size(); i++){
      for(double t = 0; t <= global_traj.times[i]; t += 0.1)
      {
        for(int j=0; j<6; j++){
          TCT_p[j] = std::pow(t, j);
        }
        auto pt  = TCT_p * global_traj.coefs[i];
        point.pose.position.x = pt[0];
        point.pose.position.y = pt[1];
        point.pose.position.z = pt[2];
        msgPath.poses.push_back(point);
      }
    }
    msgGlobalPath_.push(msgPath);
    FLAG_globalChange_ = true;
  }

  // TODO: 
  void getLocalTrj(){}

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
  ros::Publisher  visualPathPub_;
  ros::Publisher  odomMarkPub_;
  ros::Publisher  detectMarkPub_;
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
  std::queue<nav_msgs::Path> msgGlobalPath_, msgLocalPath_;
};