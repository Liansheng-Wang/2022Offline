#pragma once


#include <ros/ros.h>
#include <common/base.h>
#include <control/actuator.h>
#include <control/controller.h>
#include <geometry_msgs/PoseArray.h>
#include <visual/visualTools.h>
// #include <plan_env/voxelmap.h>

#include <plan/depthAvoid.h>
#include <plan/planner.h>
#include <Eigen/Core>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <pcl_conversions/pcl_conversions.h>

namespace Competition{

class CompetitionFSM{
public:
  // 预设的目标点
  int MissionIndex_ = 0, MissionNums_;
  std::vector<bool>   isFree;   // 是否需要避障
  std::vector<double> poses_;    // 圆环的点
  std::vector<int>    types;    // 0 红， 1 蓝， 2 墙， 3 半圆， 4插补点
  std::vector<Eigen::Vector3d> waypoints_;
  Eigen::Vector3d bias;         // 这个是起飞点的问题造成一个整体偏置

  // 视觉检测的部分
  geometry_msgs::PoseArray DetectResult_;           // 检测直接给来的 raw值
  Eigen::Vector3d MissionPoint_, GlobalDetct_;      // 筛选出来，转化过的全局点。并且用这个全局点，判断是否Cross
  bool Flag_Detect_ = false;
  bool Flag_Replan_Local_  = true;
  bool Flag_Replan_Global_ = true;
  bool Flag_Obstacle_      = false;
  bool Flag_getDepth_      = false;

  // 无人机状态
  State uavPose_;
  enum FSM_STATE
  {
    INIT,              // 规划一个全局轨迹
    GLOBAL_TRAJ,       // 执行全局轨迹
    REPLAN_TRAJ,       // 规划局部轨迹
    LOCAL_TRAJ,        // 执行局部轨迹
    OBSTACLE           // 避障点，用 A 星在地图中搜索
  };
  FSM_STATE fsm_state_;

  // ROS 相关
  ros::NodeHandle nh_;
  ros::Subscriber detectSub_;
  ros::Subscriber depthSub_;
  ros::Publisher  cloudPub_;
  ros::Rate loopRate_;

  Planner planner_;
  Actuator actuator_;
  VisualTool visualtool_;

  cv::Mat img_depth_;   
  cv_bridge::CvImagePtr cv_ptr_;

/* 全局函数 */
public:
  CompetitionFSM(ros::NodeHandle& nh): loopRate_(30), nh_(nh){
    InitWayPoints();
    visualtool_.setTargetMarker(waypoints_, poses_, types);
    detectSub_ = nh_.subscribe<geometry_msgs::PoseArray>("/paddleseg/detect", 1, &CompetitionFSM::DetectCb, this);
    depthSub_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 100 , &CompetitionFSM::imageDepthCallback, this);
    cloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/test/points", 1);
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

  void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg){
  // void imageDepthCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
    if(Flag_getDepth_){
      try
      {
          //  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
          //  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
          cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
      //  img_sub = cv_ptr->image;
      cv_ptr_->image.copyTo(img_depth_);
    }
  }

  ~CompetitionFSM(){}

  void InitWayPoints()
  {
    // poses_ 中小于 -200 的是不约束，该点速度方向的点
    waypoints_.push_back({3.0,    0.0,  1.5}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);  
    waypoints_.push_back({4.0,    0.0,  1.5}); poses_.push_back(0);     isFree.push_back(true);  types.push_back(0);  // 1 号环         1
    waypoints_.push_back({4.3,    0.0,  1.5}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);
    waypoints_.push_back({8.0,   -0.4,  1.8}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4); 
    waypoints_.push_back({9.0,   -0.4,  1.8}); poses_.push_back(0);     isFree.push_back(true);  types.push_back(0);  // 2 号环         4
    waypoints_.push_back({13.0,  -1.08, 1.4}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);
    waypoints_.push_back({14.285,-1.18, 1.2}); poses_.push_back(0);     isFree.push_back(true);  types.push_back(2);  // 第一个墙面      6
    waypoints_.push_back({14.585,-1.18, 1.2}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);
    waypoints_.push_back({22.8,  -0.8,  1.5}); poses_.push_back(20);    isFree.push_back(false); types.push_back(0);  // 11 号环        8
    // waypoints_.push_back({24.4,   2.6,  1.5}); poses_.push_back(90);  isFree.push_back(true); types.push_back(0);   // 12-1 号环     9 
    waypoints_.push_back({27.90,  2.8,  1.5}); poses_.push_back(90);    isFree.push_back(true);  types.push_back(0);  // 12-2 号环      9
    waypoints_.push_back({22.80,  6.54, 1.5}); poses_.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 13 号环        10
    waypoints_.push_back({17.72,  6.05, 1.35});poses_.push_back(-404);  isFree.push_back(false); types.push_back(4);  // 插补一个避障的点 11
    waypoints_.push_back({14.285, 5.94, 1.2}); poses_.push_back(180);   isFree.push_back(false); types.push_back(2);  // 第二个墙面      12
    waypoints_.push_back({9.0,    5.75, 1.5}); poses_.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 8 号环动态      13
    waypoints_.push_back({3.9,    4.6,  1.5}); poses_.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 9 号环         14   
    waypoints_.push_back({-1.0,   4.6,  1.5}); poses_.push_back(180);   isFree.push_back(true);  types.push_back(3);  // 10 号异型环     15
    waypoints_.push_back({-1.9,   4.6,  1.5}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);  // 降落点         16
    // 因为起飞位置造成的 bias, 需要现场修正呢。
    bias << 0.45, 0, -0.36;
    for(int i = 0; i < waypoints_.size(); i++){
      waypoints_[i] += bias;
    }
    waypoints_.shrink_to_fit(); poses_.shrink_to_fit(); 
    isFree.shrink_to_fit();    types.shrink_to_fit();
    MissionNums_ = waypoints_.size();
  }

  // 最大似然。暂时没有用到，但是应该要用的。
  int maxLikelihood(Eigen::Vector3d point){
    int index = 0;
    double min_dis = (waypoints_[index] - point).norm();
    for(int i = 1; i < waypoints_.size(); i++)
    {
      double dis = (waypoints_[i] - point).norm();
      if(dis < min_dis){
        min_dis = dis;
        index = i;
      }
    }
    return index;
  }

  // 借用了全局定位的 index
  bool isCross(double thresh = 0.1){
    // 本来应该是一个法向正负的问题，但是我想偷懒d
    if(MissionIndex_ < 9){
      // 前 4 个
      return uavPose_.pt[0] + thresh > waypoints_[MissionIndex_][0];
    }
    else if(MissionIndex_ < 10){
      // 12号环
      return uavPose_.pt[1] + thresh > waypoints_[MissionIndex_][1];
    }else{
      // 返回回来的那些点
      return uavPose_.pt[0] - thresh < waypoints_[MissionIndex_][0];
    }
  }
  
  void transDetect(){
    Eigen::Vector3d temp;
    double siny = sin(uavPose_.rpy[2]);
    double cosy = cos(uavPose_.rpy[2]);
    temp[0] = MissionPoint_[0] * cosy - MissionPoint_[1] * siny;
    temp[1] = MissionPoint_[0] * siny + MissionPoint_[1] * cosy;
    temp[2] = MissionPoint_[2];
    GlobalDetct_ = uavPose_.pt + temp;
  }

/* 状态检测切换线程函数 */

/* 主运行函数 */
  void run()
  {
    mavros_msgs::PositionTarget cmdPVAY;
    geometry_msgs::TwistStamped cmdVel;

    /* Step1: 等待手动切换 OFFBOARD 起飞！ */ 
    actuator_.waitTakeoff();
    actuator_.takeoff(1.0);         // 起飞这个距离，实际到达高度应该到 0.85 左右。已经足够了

    /* Step2: 根据任务规划全局轨迹，作为一个参考 */
    ros::spinOnce();
    State startState, endState;
    std::vector<Eigen::Vector3d> waypoints;
    std::vector<double> poses;

    /* Step3：前三个任务点直接一波过了  */
    while(ros::ok()){
      ros::spinOnce();
      uavPose_ = actuator_.getPose();
      if(Flag_Replan_Global_){
        waypoints.clear(); poses.clear();
        for(int i = MissionIndex_;  i< MissionNums_; i++){
          waypoints.push_back(waypoints_[i]);
          poses.push_back(poses_[i]);
        }
        startState = uavPose_;
        startState.vel = {0.3, 0, 0};
        endState.pt = waypoints.back();
        planner_.planGlobalTraj(startState, endState, waypoints, poses);
        visualtool_.setGlobalTrj(planner_.globalTraj_);
        Flag_Replan_Global_ = false;
      }
      while(ros::ok()){
        ros::spinOnce();
        if(Flag_Detect_){
          transDetect();
        }
        uavPose_ = actuator_.getPose();
        State pt2follow = planner_.getGlobalPathPoint(ros::Time::now().toSec());
        if(cmdPVAY.position.x > waypoints_[7][0]){
          MissionIndex_ = 8;
          break;
        }
        cmdPVAY.header.frame_id = cmdPVAY.FRAME_LOCAL_NED;
        cmdPVAY.coordinate_frame = 1;
        cmdPVAY.header.stamp = ros::Time::now();
        cmdPVAY.position.x = pt2follow.pt[0];
        cmdPVAY.position.y = pt2follow.pt[1];
        cmdPVAY.position.z = pt2follow.pt[2];
        // cmdPVAY.yaw = atan2(pt2follow.vel[1], pt2follow.vel[0]);
        actuator_.setPVAY(cmdPVAY);

        // if(isCross()){
        //   std::cout <<"Crossed: " << MissionIndex_ << std::endl;
        //   MissionIndex_++;
        //   break;
        // }
        loopRate_.sleep();
      }

      if(MissionIndex_ > 7){
        break;
      }
      loopRate_.sleep();
    }
    
    // /* Step4：动态障碍物  */
    Flag_getDepth_ = true;
    DepthAvoid deavoid;
    while(ros::ok()){
      ros::spinOnce();
      if (img_depth_.empty()){
        ROS_ERROR("Null Depth Image!");
        loopRate_.sleep();
        continue;
      }

      PointCloud::Ptr cloud(new PointCloud);
      deavoid.depth2points(img_depth_, cloud);

      geometry_msgs::Point local_target;
      deavoid.getLocalTarget(img_depth_, local_target);

      std::cout << " local_target:  " << local_target << std::endl;



      loopRate_.sleep();
    }
    Flag_getDepth_ = false;




  }  // run() END
}; // class END
  
};
