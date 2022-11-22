#pragma once


#include <common/base.h>
#include <cmath>
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
  bool planGlobalTraj(const State& curState, const State& endState, const std::vector<Eigen::Vector3d>& waypoints){
    std::vector<Eigen::Vector3d> points;
    points.push_back(curState.pt);
    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (curState.pt - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // 在全局轨迹点中插补一些新的点进来
    std::vector<Eigen::Vector3d> inter_points;
    double dist_thresh = std::max(total_len / 8, 4.0);
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // 这一步按照 ego_planner 来的： 构建一个 全局位置矩阵。
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];
    
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd times(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      times(i) = (pos.col(i + 1) - pos.col(i)).norm() / (UP::MaxVel);
    }

    times(0) *= 2.0;
    times(times.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, curState.vel, endState.vel, curState.acc, endState.acc, times);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_traj_gen(curState, endState, times(0));
    else
      return false;
    
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
