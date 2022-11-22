#pragma once


#include <ros/ros.h>
#include <common/base.h>
#include <control/actuator.h>
#include <control/controller.h>
#include <geometry_msgs/PoseArray.h>

#include <plan/planner.h>
#include <Eigen/Core>

namespace Competition{

/* 全局变量 */

  // 预设的目标点
  int MissionIndex_ = 0, MissionNums_;
  std::vector<bool>   isFree;   // 是否需要避障
  std::vector<double> poses;    // 圆环的点
  std::vector<int>    types;    // 0 红， 1 蓝， 2 墙， 3 半圆， 4插补点
  std::vector<Eigen::Vector3d> waypoints;

  // 视觉检测的部分
  geometry_msgs::PoseArray DetectResult_;           // 检测直接给来的 raw值
  Eigen::Vector3d MissionPoint_, GlobalDetct_;      // 筛选出来，转化过的全局点。并且用这个全局点，判断是否Cross
  bool Flag_Detect_ = false;

/* 全局函数 */
  void InitWayPoints()
  {
    waypoints.push_back({4.0,    0.0,  1.5}); poses.push_back(0);     isFree.push_back(true);  types.push_back(0);  // 1 号环         0
    waypoints.push_back({9.0,   -0.4,  1.8}); poses.push_back(0);     isFree.push_back(true);  types.push_back(0);  // 2 号环         1
    waypoints.push_back({14.285,-1.18, 1.2}); poses.push_back(0);     isFree.push_back(true);  types.push_back(2);  // 第一个墙面      2
    waypoints.push_back({22.8,  -0.8,  1.5}); poses.push_back(20);    isFree.push_back(false); types.push_back(0);  // 11 号环        3
    // waypoints.push_back({24.4,   2.6,  1.5}); poses.push_back(90);  isFree.push_back(true); types.push_back(0);   // 12-1 号环     4 
    waypoints.push_back({27.90,  2.8,  1.5}); poses.push_back(90);    isFree.push_back(true);  types.push_back(0);  // 12-2 号环      4
    waypoints.push_back({22.80,  6.54, 1.5}); poses.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 13 号环        5
    waypoints.push_back({17.72,  6.05, 1.35});poses.push_back(180);   isFree.push_back(false); types.push_back(4);  // 插补一个避障的点 6
    waypoints.push_back({14.285, 5.94, 1.2}); poses.push_back(180);   isFree.push_back(false); types.push_back(2);  // 第二个墙面      7
    waypoints.push_back({9.0,    5.75, 1.5}); poses.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 8 号环动态      8
    waypoints.push_back({3.9,    4.6,  1.5}); poses.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 9 号环         9   
    waypoints.push_back({-1.0,   4.6,  1.5}); poses.push_back(180);   isFree.push_back(true);  types.push_back(3);  // 10 号异型环    10
    waypoints.push_back({-1.9,   4.6,  1.0}); poses.push_back(180);   isFree.push_back(true);  types.push_back(4);  // 降落点         11

    waypoints.shrink_to_fit(); poses.shrink_to_fit(); 
    isFree.shrink_to_fit();    types.shrink_to_fit();
    MissionNums_ = waypoints.size();
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

  // 借用了全局定位的 index
  bool isCross(State& uavState, double thresh){
    Eigen::Vector3d pt = uavState.pt;
    return (pt - MissionPoint_).norm() < thresh;
  }
  
  void DetectCb(const geometry_msgs::PoseArray::ConstPtr& msg){
    DetectResult_ = *msg;
    // 根据任务点类别挑选结果。
    for(auto result : DetectResult_.poses){
      if(result.position.x > 1){
        Flag_Detect_ = false;
        return;
      }
      if(int(result.orientation.w) == types[MissionIndex_]){
        MissionPoint_[0] = result.orientation.x;
        MissionPoint_[1] = result.orientation.y;
        MissionPoint_[2] = result.orientation.z;
      }
    }
    Flag_Detect_ = true;
  }

  void transDetect(){
    if (Flag_Detect_){
      Eigen::Vector3d temp;


    }
  }

/* 主运行函数 */
  void run(ros::NodeHandle& nh)
  {
    InitWayPoints();     // 初始化预设的位姿。然后通过detect去更新
    ros::Subscriber detectSub = nh.subscribe<geometry_msgs::PoseArray>("/mavros/local_position/odom", 1, &DetectCb);
    
    /* Step1: 等待手动切换 OFFBOARD 起飞！ */ 
    Actuator actuator; Planner planner;
    Controller controller;
    State uavPose, endState;
    
    std::vector<Eigen::Vector3d> targets;
    int segments = 1;
    controller.init(actuator);     // 控制器传入执行器指针。可以设置执行器指令。
    actuator.waitTakeoff();
    actuator.takeoff(1.5);

    /* Step2：获取状态准备规划  */   // 一个圆环一个圆环写，比较容易调试。
    
    // MissionIndex_ = 0； 1 号任务点
    while(ros::ok()){
      ros::spinOnce();
      transDetect();
      uavPose = actuator.getPose();   // 开始状态
      endState.vel = {cos(poses[MissionIndex_]), sin(poses[MissionIndex_]), 0};
      endState.pt = waypoints[MissionIndex_] + endState.vel * 0.6;
      endState.vel = endState.vel * UP::MaxVel;
      endState.acc = Eigen::Vector3d::Zero();
      targets.clear();
      segments = 2;
      if(Flag_Detect_){
        targets.push_back(GlobalDetct_);
      }else{
        targets.push_back(waypoints[MissionIndex_]);
      }
      
      if(isFree[MissionIndex_]){
        // TODO: 这个得分情况处理，如果检测到了，怎么样。没检测到，怎么样？
        planner.planGlobalTraj(uavPose, endState, targets, segments);
      }
      

    }
    
    








    







    
  }
};