#pragma once;

#include <param.h>
#include <common.h>
#include <vector>
#include <Eigen/Core>

namespace UP = UAVparam;

class Planner
{
public:
  Planner(){
    initConstant();
  }

  void plan(const State& start, const State& end)
  {
    // model： Ax = b 求解系数 x;
    Eigen::Matrix<double, 6, 3> constant_b;
    constant_b.row(0) = start.pt.transpose();
    constant_b.row(1) = start.vel.transpose();
    constant_b.row(2) = start.acc.transpose();
    constant_b.row(3) = end.pt.transpose();
    constant_b.row(4) = end.vel.transpose();
    constant_b.row(5) = end.acc.transpose();
    PathCoe_ = Polynomial_ * constant_b;

    totalTime_ = (end.pt - start.pt).norm()   / UP::MaxVel + 
                 (end.vel - start.vel).norm() / UP::MaxAcc;
  }


  Eigen::Matrix<double, 6, 3> getPathCoe()
  {
    return PathCoe_;
  }

private:
  Eigen::Matrix<double, 6, 6> Polynomial_;   // 归一化之后的求解矩阵
  Eigen::Matrix<double, 6, 3> PathCoe_;      // 归一化之后的路径系数
  std::vector<double> TCT_;                  // time coefficient table -> TCT
  double totalTime_;

private:
  void initConstant(){
    Polynomial_ << 1, 0, 0, 0, 0, 0,
                   0, 1, 0, 0, 0, 0,
                   0, 0, 0.5, 0, 0, 0,
                  -10, -6, -1.5, 10, -4, 0.5,
                   15, 8, 1.5, -15, 7, -1,
                  -6, -3, -0.5, 6, -3, 0.5;
  }

  void caluTime(double t)
  {
    for(int i=0; i<6; i++)
    {
      TCT_.push_back(std::pow(t, i));
    }
  }

};