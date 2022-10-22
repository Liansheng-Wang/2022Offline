#pragma once

#include <param.h>
#include <common.h>
#include <tuple>
#include <vector>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <mutex>

namespace UP = UAVparam;

class Planner
{
public:
  Planner(){
    initConstant();
    pathPub_ = nh_.advertise<nav_msgs::Path>("/visual/planner/path", 1);
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
    ros::Rate loopRate(10);
    while(ros::ok() && FLAG_running_){
      if(FLAG_pathChange_){
        reCalPath();
      }
      if(!msgPath_.poses.empty()){
        pathPub_.publish(msgPath_);
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

    std::cout << "constant_b:   " << std::endl;
    std::cout << constant_b << std::endl;

    PathCoe_ = Polynomial_ * constant_b;

    FLAG_pathChange_ = true;
  }

  State getPathPoint(double rr_t) // real relative time 真实的相对时间
  {
    State pathPoint;
    if(rr_t > totalTime_){
      ROS_ERROR("---> Real time is oversize than totalTime_!");
      return pathPoint;
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
  nav_msgs::Path msgPath_;
  std::thread* visulThread_ = nullptr;
  std::atomic_bool FLAG_pathChange_;
  std::atomic_bool FLAG_running_;
  std::mutex pathInfoMut_;

}; 
// Class Planner END ========================================================