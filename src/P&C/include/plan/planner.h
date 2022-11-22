#pragma once


#include <common/base.h>
#include <mutex>
#include <tuple>
#include <vector>
#include <thread>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>


namespace UP = UAVparam;


class Planner
{
public:
  Planner(){
    initConstant();
  }

  ~Planner(){                        
    std::cout<< "\033[32m ---> Planner    closed ^_^ ! \033[0m" << std::endl;
  }

public:
  typedef std::unique_ptr<Planner> Ptr;
  PolynomialTraj localTraj_;
  PolynomialTraj globalTraj_;

  // 如果只有起点和终点的话，就是一段，如果多段的话，targets 是中间的路点。
  bool planGlobalTraj(State& curState, State& endState, std::vector<Eigen::Vector3d> targets, int multiSeg){
    if(multiSeg > 1){
      Eigen::MatrixXd wps(3, multiSeg+1);
      Eigen::VectorXd times(multiSeg);
      wps.col(0) = curState.pt;
      for(int i = 1; i < targets.size(); i++)
      {
        wps.col(i) = targets[i-1];
        times(i-1) = (wps.col(i) - wps.col(i-1)).norm() / UP::MaxVel;
      }
      wps.col(multiSeg) = endState.pt;
      times(multiSeg-1) = (wps.col(multiSeg) - wps.col(multiSeg - 1)).norm() / UP::MaxVel;
      globalTraj_ = PolynomialTraj::minSnapTraj(wps, curState.vel, endState.vel, 
                    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times);

    }else{
      double distance = (curState.pt - endState.pt).norm();
      double time = distance / UP::MaxVel;
      globalTraj_ = PolynomialTraj::one_traj_gen(curState, endState, time);
    }
    globalTraj_.setStartTime();
    return true;
  }

  // 传入的参数 t 是全局的时间
  State getGlobalPathPoint(double t){
    globalTraj_.evaluate(t);
    return globalTraj_.globalState_;
  }

  bool planLocalTraj(){


    return true;
  }


  // State getLocalPathPoint(double t){

    
  // }


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
            0, 0, 2, 0, 0, 0,
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
  }

  State getPathPoint(double rr_t)  // real relative time 真实的相对时间
  {
    State pathPoint;
    if(rr_t > totalTime_){
      ROS_INFO("\033[33m ---> Need replan! \033[0m");
    }
    caluTime(rr_t);

    pathPoint.pt  = TCT_p_ * PathCoe_;  // 1x6 * 6x3 = 1x3 
    pathPoint.vel = TCT_v_ * PathCoe_;
    pathPoint.acc = TCT_a_ * PathCoe_;

    return pathPoint;
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

  void initConstant(){
    TCT_p_[0] = 1;
    TCT_v_[0] = 0; TCT_v_[1] = 1;
    TCT_a_[0] = 0; TCT_a_[0] = 0; TCT_a_[1] = 2;
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
};

// Class Planner END ========================================================
