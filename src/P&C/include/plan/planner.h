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
    const std::vector<Eigen::Vector3d>& waypoints, const std::vector<double>& poses)
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

    if (pos.cols() >= 3)
      globalTraj_ = PolynomialTraj::minSnapTraj(pos, curState.vel, endState.vel, curState.acc, endState.acc, times, posesfull);
    else if (pos.cols() == 2)
      globalTraj_ = PolynomialTraj::one_traj_gen(curState, endState, times(0));
    else
      return false;
    
    // 重新分配一下时间：
    std::cout << std::endl << std::endl;
    for (int i = 0; i < pt_num - 1; ++i)
    {
      double dis = globalTraj_.getPathLen(i);
      times(i) = dis / (UP::MaxVel);
      // std::cout << i  << "  dis:  "<< dis << "  time:  " << times(i) << std::endl;
    }
    times(0) += 2 * UP::MaxVel / UP::MaxAcc;
    times(times.rows() - 1) += 2 * UP::MaxVel / UP::MaxAcc;

    if (pos.cols() >= 3)
      globalTraj_ = PolynomialTraj::minSnapTraj(pos, curState.vel, endState.vel, curState.acc, endState.acc, times, posesfull);
    else if (pos.cols() == 2)
      globalTraj_ = PolynomialTraj::one_traj_gen(curState, endState, times(0));
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

  State getLocalTarget(State& uavState){
    double t;
    double planning_horizen = 5.0;   // 局部滑窗的长度
    double t_step = planning_horizen / 20 / UP::MaxVel;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = globalTraj_.last_progress_time_; t < globalTraj_.getTotalTime(); t += t_step)
    {
      Eigen::Vector3d pos_t = globalTraj_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

  double getGlobalTotalTime(){
    return globalTraj_.getTotalTime();
  }

  bool planLocalTraj(){
    

    return true;
  }

  // TODO:
  // State getLocalPathPoint(double t){

    
  // }

};

// Class Planner END ========================================================
