/***
 * 
 * 
*/


#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <ros/ros.h>

// 一些无人机参数相关的东西
namespace UAVparam{
  double MaxVel = 1.0;
  double MaxAcc = 1.0;
  double MaxYawRate = M_PI;

  void LoadFromYaml(ros::NodeHandle& nh)
  {
    // TODO: 晚上从 launch 中加载参数


  }
};

// 无人机飞行状态：
struct State
{
  Eigen::Vector3d pt;      // 全局的位置
  Eigen::Vector3d vel;     // 全局的速度  
  Eigen::Vector3d acc;     // 局部的加速度？
  Eigen::Vector3d rpy;     // Yaw是全局的，r p应该是机体的

  State(){
    pt = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    rpy = Eigen::Vector3d::Zero();
  }
};

// 多项式轨迹的描述方法
struct PolyPath{
  int order;
  double tt_t;            // total time
  Eigen::MatrixXd coef;   // 各个轴上的维度是列向量

  PolyPath(int ord){
    order = ord;
    tt_t = 0;
    coef = Eigen::MatrixXd(order+1, 3); 
  }
};

// 多段多项式
struct MultiPolyPath
{
  int order;
  std::vector<double> times;
  std::vector<Eigen::MatrixXd> coefs;

  MultiPolyPath(int ord){
    order = ord;
  }
};

