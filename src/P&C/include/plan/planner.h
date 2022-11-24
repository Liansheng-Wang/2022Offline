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
  Planner(){}

  ~Planner(){                        
    std::cout<< "\033[32m ---> Planner    closed ^_^ ! \033[0m" << std::endl;
  }

public:
  typedef std::unique_ptr<Planner> Ptr;
  PolynomialTraj localTraj_;
  PolynomialTraj globalTraj_;

  bool planGlobalTraj(const State& curState, const State& endState, 
    const std::vector<Eigen::Vector3d>& waypoints, const std::vector<double>& poses, bool global = true)
  {
    std::vector<double> posesfull;
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

    // 在全局轨迹点中插补一些新的点进来。
    // FIXME: 这种线性插补的方式很有问题。
    std::vector<Eigen::Vector3d> inter_points;
    double dist_thresh = std::max(total_len / 8, 4.0);
    // dist_thresh = std::min(dist_thresh, 5.0);
    // double dist_thresh = 4.0;
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      posesfull.push_back(poses.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
          posesfull.push_back(-201);     // 设阈值为 -200，小于 -200为pose自由的点
        }
      }
    }

    inter_points.push_back(points.back());
    posesfull.push_back(poses.back());

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
      // std::cout << i  << "  dis:  "<< (pos.col(i + 1) - pos.col(i)).norm() << "  time:  " << times(i) << std::endl;
    }

    times(0) += 2 * UP::MaxVel / UP::MaxAcc;
    times(times.rows() - 1) += 2 * UP::MaxVel / UP::MaxAcc;

    PolynomialTraj ployTraj;
    if (pos.cols() >= 3)
      ployTraj = PolynomialTraj::minSnapTraj(pos, curState.vel, endState.vel, curState.acc, endState.acc, times, posesfull);
    else if (pos.cols() == 2)
      ployTraj = PolynomialTraj::one_traj_gen(curState, endState, times(0));
    else
      return false;
    
    // 重新分配一下时间：
    for (int i = 0; i < pt_num - 1; ++i)
    {
      double dis = ployTraj.getPathLen(i);
      times(i) = dis / (UP::MaxVel);
      // std::cout << i  << "  dis:  "<< dis << "  time:  " << times(i) << std::endl;
    }
    times(0) += 2 * UP::MaxVel / UP::MaxAcc;
    times(times.rows() - 1) += 2 * UP::MaxVel / UP::MaxAcc;

    if (pos.cols() >= 3)
      ployTraj = PolynomialTraj::minSnapTraj(pos, curState.vel, endState.vel, curState.acc, endState.acc, times, posesfull);
    else if (pos.cols() == 2)
      ployTraj = PolynomialTraj::one_traj_gen(curState, endState, times(0));
    else
      return false;

    ployTraj.setStartTime();
    ployTraj.last_progress_time_ = 0;

    globalTraj_ = ployTraj;

    return true;
  }

  // 传入的参数 t 是全局的时间
  State getGlobalPathPoint(double t){
    globalTraj_.evaluate(t);
    return globalTraj_.globalState_;
  }

  State getLocalPathPoint(double t){
    localTraj_.evaluate(t);
    return localTraj_.globalState_;
  }

  State getLocalTarget(State& uavState){
    State localState;
    double t;
    double planning_horizen = 7.0;   // 局部滑窗的长度
    double t_step = planning_horizen / 30 / UP::MaxVel;  // 这个数字大概是 0.1s
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = globalTraj_.last_progress_time_; t < globalTraj_.getTotalTime(); t += t_step)
    {
      Eigen::Vector3d pos_t = globalTraj_.getPosition(t);
      double dist = (pos_t - uavState.pt).norm();

      if (t < globalTraj_.last_progress_time_ + 1e-5 && dist > planning_horizen)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return localState;  // FIXME: 直接这样的话，这里肯定是有问题的。
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen)
      {
        localState.pt = pos_t;
        globalTraj_.last_progress_time_ = dist_min_t;
        break;
      }
    }

    if (t > globalTraj_.getTotalTime())     // Last global point
    {
      localState.pt = globalTraj_.getEndPoint();
    }

    if ((globalTraj_.getEndPoint() - localState.pt).norm() < (UP::MaxVel * UP::MaxVel) / (2 * UP::MaxAcc))
    {
      localState.vel = Eigen::Vector3d::Zero();
    }
    else
    {
      localState.vel = globalTraj_.getVelocity(t);
    }
    return localState;
  }

  double getGlobalTotalTime(){
    return globalTraj_.getTotalTime();
  }

  bool planLocalTraj(const State& curState, const State& endState, 
    const std::vector<Eigen::Vector3d>& waypoints, const std::vector<double>& poses){

    std::vector<double> posesfull;
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

    std::vector<Eigen::Vector3d> inter_points;
    double dist_thresh = std::max(total_len / 8, 4.0);
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      posesfull.push_back(poses.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
          posesfull.push_back(-404); 
        }
      }
    }

    inter_points.push_back(points.back());
    posesfull.push_back(poses.back());

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

    PolynomialTraj ployTraj;
    if (pos.cols() >= 3)
      ployTraj = PolynomialTraj::minSnapTraj(pos, curState.vel, endState.vel, curState.acc, endState.acc, times, posesfull);
    else if (pos.cols() == 2)
      ployTraj = PolynomialTraj::one_traj_gen(curState, endState, times(0));
    else
      return false;

    for (int i = 0; i < pt_num - 1; ++i)
    {
      double dis = ployTraj.getPathLen(i);
      times(i) = dis / (UP::MaxVel);
    }

    if (pos.cols() >= 3)
      ployTraj = PolynomialTraj::minSnapTraj(pos, curState.vel, endState.vel, curState.acc, endState.acc, times, posesfull);
    else if (pos.cols() == 2)
      ployTraj = PolynomialTraj::one_traj_gen(curState, endState, times(0));
    else
      return false;

    ployTraj.setStartTime();
    ployTraj.last_progress_time_ = 0;

    localTraj_ = ployTraj;

    return true;
  }
  

  // 单端的局部轨迹多项式
  bool planLocalTraj(const State& curState, const State& endState){
    double time = (curState.pt  - endState.pt).norm()  / UP::MaxVel + 
                  (curState.vel - endState.vel).norm() / UP::MaxAcc;
    localTraj_ = PolynomialTraj::one_traj_gen(curState, endState, time);

    // 时间重新分配一下：
    double dis = localTraj_.getPathLen(0);
    time = dis / UP::MaxVel + (curState.vel - endState.vel).norm() / UP::MaxAcc;
    localTraj_ = PolynomialTraj::one_traj_gen(curState, endState, time);
    localTraj_.last_progress_time_ = 0;
    localTraj_.setStartTime();
    return true;
  }
};

// Class Planner END ========================================================
