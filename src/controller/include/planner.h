#pragma once

#include <uav_control/param.h>
#include <common.h>
#include <tuple>
#include <vector>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

namespace UP = UAVparam;

class Planner
{
public:
  Planner(){
    initConstant();
    pathPub_   = nh_.advertise<nav_msgs::Path>("/visual/planner/path", 1);
    targetPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visual/planner/target", 1);
    FLAG_pathChange_ = false;
    FLAG_running_ = true;
    msgPath_.header.frame_id = "map";
    visulThread_ = new std::thread(std::bind(&Planner::visualThread, this));
  }

  ~Planner(){
    FLAG_running_ = false;
    visulThread_->join();
    delete visulThread_;                           
    ROS_INFO("\033[32m ---> Planner    closed ^_^ ! \033[0m");
  }

  void visualThread(){
    ros::Rate loopRate(2);
    while(ros::ok() && FLAG_running_){
      if(FLAG_pathChange_){
        reCalPath();
      }
      if(!msgPath_.poses.empty()){
        pathPub_.publish(msgPath_);
      }
      if(!msgMarkers.markers.empty()){
        targetPub_.publish(msgMarkers);
      }
      loopRate.sleep();
    }
  }

  // 重新计算 Path
  void reCalPath(){
    msgPath_.poses.clear();
    double tt_time;
    Eigen::Matrix<double, 6, 3> PathCoe;
    {
      std::lock_guard<std::mutex> tempMut(pathInfoMut_);
      tt_time = totalTime_;
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

  // wps -> waypoints, ps -> pose
  void setTargetMarker(std::vector<Eigen::Vector3d>& wps, std::vector<double>& ps){
    msgMarkers.markers.clear();
    for(int i=0; i < wps.size(); i++){
      auto tf_q = tf::createQuaternionFromRPY(0, M_PI_2, ps[i]/180*M_PI);
      visualization_msgs::Marker marker;
      marker.mesh_resource = std::string("package://controller/configs/meshes/circle.mesh");
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
      marker.scale.x = 0.001;
      marker.scale.y = 0.001;
      marker.scale.z = 0.001;
      msgMarkers.markers.push_back(marker);
    }
  } 

  void plan(const State& start, const State& end)
  {
    totalTime_ = (end.pt - start.pt).norm()   / UP::MaxVel + 
                 (end.vel - start.vel).norm() / UP::MaxAcc;     // 粗略的时间计算，按道理应该是不准确的
    double tt[6];
    for(int i = 0; i < 6; i++){
      tt[i] = pow(totalTime_,i);
    }

    Eigen::Matrix<double, 6, 6> poly;
    poly << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            tt[0], tt[1], tt[2], tt[3], tt[4], tt[5],
            0, 1, 2*tt[1], 3*tt[2], 4*tt[3], 5*tt[4],
            0, 0, 2, 6*tt[1], 12*tt[2], 20*tt[3];
    Polynomial_ = poly.inverse();
    // model： Ax = b 求解系数 x;
    Eigen::Matrix<double, 6, 3> constant_b;
    constant_b.row(0) = start.pt.transpose();
    constant_b.row(1) = start.vel.transpose();
    constant_b.row(2) = start.acc.transpose();
    constant_b.row(3) = end.pt.transpose();
    constant_b.row(4) = end.vel.transpose();
    constant_b.row(5) = end.acc.transpose();

    ROS_INFO("\033[32m ---> Planning.... \033[0m");
    std::cout << constant_b << std::endl;

    PathCoe_ = Polynomial_ * constant_b;

    FLAG_pathChange_ = true;
  }

  State getPathPoint(double rr_t) // real relative time 真实的相对时间
  {
    State pathPoint;
    if(rr_t > totalTime_){
      ROS_INFO("\033[33m ---> Need replan! \033[0m");
    }
    caluTime(rr_t);
    {
      std::lock_guard<std::mutex> tempMut(pathInfoMut_);
      pathPoint.pt  = TCT_p_ * PathCoe_;  // 1x6 * 6x3 = 1x3 
      pathPoint.vel = TCT_v_ * PathCoe_;
      pathPoint.acc = TCT_a_ * PathCoe_;
    }
    return pathPoint;
  }

  void initConstant(){
    TCT_p_[0] = 1;
    TCT_v_[0] = 0; TCT_v_[1] = 1;
    TCT_a_[0] = 0; TCT_a_[0] = 0; TCT_a_[1] = 2;
  }

  void caluTime(double t){
    for(int i=1; i<6; i++){
      TCT_p_[i] = std::pow(t, i);
    }
    for(int i=2; i<6; i++){
      TCT_v_[i] = i * TCT_p_[i-1];
    }
    for(int i=3; i<6; i++){
      TCT_a_[i] = i * TCT_v_[i-1];
    }
  }

  Eigen::Matrix<double, 6, 3> getPath(){
    return PathCoe_;
  }

  double getTotalTime(){
    return totalTime_;
  }


private:
  Eigen::Matrix<double, 6, 6> Polynomial_;   // 归一化之后的求解矩阵
  Eigen::Matrix<double, 6, 3> PathCoe_;      // 归一化之后的路径系数
  Eigen::Matrix<double, 1, 6> TCT_p_;        // time coefficient table -> TCT
  Eigen::Matrix<double, 1, 6> TCT_v_;   
  Eigen::Matrix<double, 1, 6> TCT_a_;
  double totalTime_;

  ros::NodeHandle nh_;
  ros::Publisher pathPub_;
  ros::Publisher targetPub_;
  nav_msgs::Path msgPath_;
  visualization_msgs::MarkerArray msgMarkers;
  std::thread* visulThread_ = nullptr;
  std::atomic_bool FLAG_pathChange_;
  std::atomic_bool FLAG_running_;
  std::mutex pathInfoMut_;

}; 
// Class Planner END ========================================================
