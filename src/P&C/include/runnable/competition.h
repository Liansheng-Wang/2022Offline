#pragma once


#include <ros/ros.h>
#include <control/controller.h>
#include <control/actuator.h>
#include <geometry_msgs/PoseArray.h>

namespace Competition{

  // 预设的目标点
  std::vector<Eigen::Vector3d> waypoints, detect_waypoints;
  std::vector<double> poses;
  void InitWayPoints()
  {
    waypoints.push_back({6.5,    0.4,  0.8}); poses.push_back(0);
    waypoints.push_back({13,     0.2,  0.8}); poses.push_back(0);
    waypoints.push_back({20,     0.2,  0.8}); poses.push_back(0);
    waypoints.push_back({27.50, -0.2,  0.8}); poses.push_back(20);
    waypoints.push_back({32.50,  3.50, 0.8}); poses.push_back(90);
    waypoints.push_back({28.00,  7.5,  0.8}); poses.push_back(160);
    waypoints.push_back({21.00,  7.5,  0.8}); poses.push_back(180);
    waypoints.push_back({14.00,  7.5,  0.8}); poses.push_back(180);
    waypoints.push_back({8.00,   7.5,  0.8}); poses.push_back(180);
    waypoints.push_back({1.00,   7.5,  0.8}); poses.push_back(180);
    waypoints.push_back({0.00,   7.5,  0.8}); poses.push_back(180);
    waypoints.shrink_to_fit(); poses.shrink_to_fit();
    detect_waypoints = waypoints;
    detect_waypoints.shrink_to_fit();
  }

  // 最大似然
  int maxLikelihood(Eigen::Vector3d point){
    int index = 0;
    double min_dis = (waypoints[index] - point).norm();
    for(int i = 1; i < waypoints.size(); i++)
    {
      double dis = (waypoints[i] - point).norm();
      if(dis < min_dis){
        min_dis = dis;
        index = i;
      }
    }
    return index;
  }

  
  // TODO: 检测这一部分，暂定
  geometry_msgs::PoseArray DetectResult_;
  void DetectCb(const geometry_msgs::PoseArray::ConstPtr& msg){
    DetectResult_ = *msg;
    

    

  }

  nav_msgs::Odometry visionOdom_;
  void visionPoseCb(const nav_msgs::Odometry::ConstPtr& msg){
    visionOdom_ = *msg;
  }

  void run(ros::NodeHandle& nh)
  {
    InitWayPoints();     // 初始化预设的位姿。然后通过detect去更新
    ros::Subscriber detectSub = nh.subscribe<geometry_msgs::PoseArray>("/mavros/local_position/odom", 1, &DetectCb);
    ros::Subscriber visionPoseSub = nh.subscribe<nav_msgs::Odometry>("/mavros/vision_pose/pose", 1, &visionPoseCb);
    
    // 等待手动切换 OFFBOARD
    Actuator actuator;
    Controller controller;
    controller.init(actuator);    // 控制器传入执行器指针。可以设置执行器指令。
    actuator.waitTakeoff();
    actuator.takeoff(1.5);

    







    
  }
};