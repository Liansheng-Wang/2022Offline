#pragma once

#include <Eigen/Core>

struct State
{
  Eigen::Vector3d pt;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  Eigen::Vector3d rpy;

  State(){
    pt = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    rpy = Eigen::Vector3d::Zero();
  }
};

