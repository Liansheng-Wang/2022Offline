#pragma once


#include <ros/ros.h>
#include <common/base.h>
#include <control/actuator.h>
#include <control/controller.h>
#include <geometry_msgs/PoseArray.h>
#include <visual/visualTools.h>
// #include <plan_env/voxelmap.h>   // FIXME: 这个文件暂时有问题

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
  Eigen::Vector3d bias;         // 这个是起飞点的问题造成一个整体偏置

  // 视觉检测的部分
  geometry_msgs::PoseArray DetectResult_;           // 检测直接给来的 raw值
  Eigen::Vector3d MissionPoint_, GlobalDetct_;      // 筛选出来，转化过的全局点。并且用这个全局点，判断是否Cross
  bool Flag_Detect_ = false;

  // 无人机状态
  State uavPose;

/* 全局函数 */
  void InitWayPoints()
  {
    // poses 中小于 -200 的是不约束，该点速度方向的点
    waypoints.push_back({4.0,    0.0,  1.5}); poses.push_back(0);     isFree.push_back(true);  types.push_back(0);  // 1 号环         0
    waypoints.push_back({9.0,   -0.4,  1.8}); poses.push_back(0);     isFree.push_back(true);  types.push_back(0);  // 2 号环         1
    waypoints.push_back({14.285,-1.18, 1.2}); poses.push_back(0);     isFree.push_back(true);  types.push_back(2);  // 第一个墙面      2
    waypoints.push_back({22.8,  -0.8,  1.5}); poses.push_back(20);    isFree.push_back(false); types.push_back(0);  // 11 号环        3
    // waypoints.push_back({24.4,   2.6,  1.5}); poses.push_back(90);  isFree.push_back(true); types.push_back(0);   // 12-1 号环     4 
    waypoints.push_back({27.90,  2.8,  1.5}); poses.push_back(90);    isFree.push_back(true);  types.push_back(0);  // 12-2 号环      4
    waypoints.push_back({22.80,  6.54, 1.5}); poses.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 13 号环        5
    waypoints.push_back({17.72,  6.05, 1.35});poses.push_back(-201);  isFree.push_back(false); types.push_back(4);  // 插补一个避障的点 6
    waypoints.push_back({14.285, 5.94, 1.2}); poses.push_back(180);   isFree.push_back(false); types.push_back(2);  // 第二个墙面      7
    waypoints.push_back({9.0,    5.75, 1.5}); poses.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 8 号环动态      8
    waypoints.push_back({3.9,    4.6,  1.5}); poses.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 9 号环         9   
    waypoints.push_back({-1.0,   4.6,  1.5}); poses.push_back(180);   isFree.push_back(true);  types.push_back(3);  // 10 号异型环    10
    waypoints.push_back({-1.9,   4.6,  1.5}); poses.push_back(-201);  isFree.push_back(true);  types.push_back(4);  // 降落点         11
    // 因为起飞位置造成的 bias, 需要现场修正呢。
    bias << 0.45, 0, -0.36;
    for(int i = 0; i < waypoints.size(); i++){
      waypoints[i] += bias;
    }
    waypoints.shrink_to_fit(); poses.shrink_to_fit(); 
    isFree.shrink_to_fit();    types.shrink_to_fit();
    MissionNums_ = waypoints.size();
  }

  // 最大似然。暂时没有用到，但是应该要用的。
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
    Eigen::Vector3d temp;
    double siny = sin(uavPose.rpy[2]);
    double cosy = cos(uavPose.rpy[2]);
    temp[0] = MissionPoint_[0] * cosy - MissionPoint_[1] * siny;
    temp[1] = MissionPoint_[0] * siny + MissionPoint_[1] * cosy;
    temp[2] = MissionPoint_[2];
    GlobalDetct_ = uavPose.pt + temp;
  }

/* 状态检测切换线程函数 */



/* 主运行函数 */
  void run(ros::NodeHandle& nh)
  {
    InitWayPoints();     // 初始化预设的位姿。然后通过detect去更新
    ros::Subscriber detectSub = nh.subscribe<geometry_msgs::PoseArray>("/paddleseg/detect", 1, &DetectCb);
    Planner planner;
    Actuator actuator;
    Controller controller;         // 控制器是想实现一个串级 PID 的
    controller.init(actuator);     // 控制器传入执行器指针。可以设置执行器指令。
    
    VisualTool visualtool;
    visualtool.setTargetMarker(waypoints, poses, types);
  
    /* Step2: 等待手动切换 OFFBOARD 起飞！ */ 
    actuator.waitTakeoff();
    actuator.takeoff(1.0);         // 起飞这个距离，实际到达高度应该到 0.85 左右。已经足够了

    /* Step3: 根据任务规划全局轨迹，作为一个参考 */
    ros::spinOnce();
    State startState, endState;
    uavPose = actuator.getPose();
    startState.pt = uavPose.pt;   // startState 和 endState 构造全是0
    endState.pt = waypoints[MissionNums_ - 1];
    planner.planGlobalTraj(startState, endState, waypoints, poses);
    visualtool.setGlobalTrj(planner.globalTraj_);

    /* Step4：获取状态准备规划  */
    std::vector<Eigen::Vector3d> localwps;
    std::vector<double> localPoses;
    while(ros::ok()){
      ros::spinOnce();
      localwps.clear();
      uavPose = actuator.getPose();
      


      if(true){    // 这个判断条件还得再想想。
        if(Flag_Detect_){
          transDetect();
          localwps.push_back(GlobalDetct_);
        }else{
          localwps.push_back(waypoints[MissionIndex_]);
        }
        localPoses.push_back(poses[MissionIndex_]);
      }
      

      if(isFree[MissionIndex_]){
        





      }else{






        
      }
    }  
  }  // run() END
};