/***
 * 
 * 下位机文件：实现执行类： actuator 
 * 该文件主要是衔接 mavros，等待控制器下发命令的工具类。
 *
*/

#pragma once

#include <string>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/HomePosition.h>

#include <common/base.h>


enum ActionFSM
{
  Init,
  Pose,
  Position,
  Velocity,
  PVAY
};


class Actuator
{
public:
  Actuator():controlFSM_(ActionFSM::Init), FLAG_running_(true), FLAG_homeSet_(false)
  {
    stateSub_     = nh_.subscribe<mavros_msgs::State>("/mavros/state",                     1, &Actuator::StateCb, this, ros::TransportHints().tcpNoDelay());
    odomSub_      = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",       1, &Actuator::PoseCb,  this, ros::TransportHints().tcpNoDelay());
    
    // odomSub_      = nh_.subscribe<nav_msgs::Odometry>("/mavros/vision_pose/pose",          1, &Actuator::PoseCb,  this, ros::TransportHints().tcpNoDelay());
    accSub_       = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data",                    1, &Actuator::IMUCb,   this, ros::TransportHints().tcpNoDelay());
    homeSub_      = nh_.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 1, &Actuator::setHomeGeoPointCB, this);
    positionPub_  = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",    10);
    velPub_       = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    pvayPub_      = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",        10);
    posePub_      = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",     10);
    landClient_   = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    armingClient_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    modeClient_   = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    threadPub_ = new std::thread(std::bind(&Actuator::publishThread, this));
  }

  ~Actuator(){
    FLAG_running_ = false;
    threadPub_->join();
    delete threadPub_;
    ROS_INFO("\033[32m ---> Controller closed ^_^ ! \033[0m");
  }

private:
  void StateCb(const mavros_msgs::State::ConstPtr& msg){
    uavState_ = *msg;
  }

  void IMUCb(const sensor_msgs::Imu::ConstPtr& msg){
    motionState_.acc = {msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                  9.8 - msg->linear_acceleration.z};
  }

  void PoseCb(const nav_msgs::Odometry::ConstPtr& msg){
    uavPose_ = *msg;
    motionState_.rpy[2] = fromQuaternion2yaw(uavPose_.pose.pose.orientation);
    double siny = sin(motionState_.rpy[2]);
    double cosy = cos(motionState_.rpy[2]);
    motionState_.pt ={uavPose_.pose.pose.position.x,
                      uavPose_.pose.pose.position.y,
                      uavPose_.pose.pose.position.z};
    motionState_.vel={uavPose_.twist.twist.linear.x * cosy - uavPose_.twist.twist.linear.y * siny,
                      uavPose_.twist.twist.linear.y * cosy + uavPose_.twist.twist.linear.x * siny,
                      uavPose_.twist.twist.linear.z};
  }

  void setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home){
    FLAG_homeSet_ = true;
  }

  double fromQuaternion2yaw(const geometry_msgs::Quaternion& q){
    double yaw = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
    return yaw;
  }

  void publishThread()
  {
    ros::Rate loopRate = ros::Rate(30);
    while(ros::ok() && FLAG_running_)
    {
      ros::spinOnce();
      switch (controlFSM_)
      {
      case Init:{
        break;
      }
      case Pose:{
        posePub_.publish(msgPose_);
        break;
      }
      case Position:{
        positionPub_.publish(msgPosition_);
        break;
      }
      case Velocity:{
        velPub_.publish(msgVel_);
        break;
      }
      case PVAY:{
        pvayPub_.publish(msgPVAY_);
        break;
      }      
      default:
        break;
      }
      loopRate.sleep();
    }
  }

public:
  void waitHomeSet(){
    ros::Rate loopRate = ros::Rate(30);
    while(ros::ok()){
      ros::spinOnce();
      if(uavState_.connected){
        break;
      }
      loopRate.sleep();
    }
    while(ros::ok()){
      ros::spinOnce();
      if(FLAG_homeSet_){
        break;
      }
      loopRate.sleep();
    }
  }

  void takeoff(double height){
    ros::Rate loopRate = ros::Rate(30);
    ros::spinOnce();
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = uavPose_.pose.pose.position.x;
    setpoint.pose.position.y = uavPose_.pose.pose.position.y;
    setpoint.pose.position.z = uavPose_.pose.pose.position.z + height;
    msgPosition_ = setpoint;
    controlFSM_ = ActionFSM::Position;
    while(ros::ok() && abs(uavPose_.pose.pose.position.z - setpoint.pose.position.z) > 0.25){
      ros::spinOnce();
      loopRate.sleep();
    }
    ROS_INFO("\033[32m ---> Take off success ! \033[0m");
  }

  bool land(){
    mavros_msgs::CommandTOL srvLand{};
    if (landClient_.call(srvLand) && srvLand.response.success)
    {
      ROS_INFO("\033[32m ---> Land Send Success! \033[0m");
      return true;
    }
    ROS_INFO("\033[31m ---> Land Send Fail! \033[0m");
    return false;
  }

  bool disarm(){
    mavros_msgs::CommandBool srvArming;
    srvArming.request.value = false;
    if (armingClient_.call(srvArming) && srvArming.response.success)
    {
      ROS_INFO("\033[32m ---> Disarmd Success! \033[0m");
      return true;
    }
    ROS_INFO("\033[31m ---> Disarmd Fail! \033[0m");
    return false;    
  }

  bool arm(){
    mavros_msgs::CommandBool srvArming;
    srvArming.request.value = true;
    if (armingClient_.call(srvArming) && srvArming.response.success){
      ROS_INFO("\033[32m ---> Armd Success! \033[0m");
      return true;
    }
    ROS_INFO("\033[31m ---> Armd Fail! \033[0m");
    return false;    
  }

  bool setMode(std::string modeName){
    ros::Rate loopRate = ros::Rate(30);
    if(modeName == "OFFBOARD"){
      ros::spinOnce();
      geometry_msgs::PoseStamped setpoint;
      setpoint.pose.position.x = uavPose_.pose.pose.position.x;
      setpoint.pose.position.y = uavPose_.pose.pose.position.y;
      setpoint.pose.position.z = uavPose_.pose.pose.position.z;
      msgPosition_ = setpoint;
      controlFSM_ = ActionFSM::Position;
      for (int i = 100; ros::ok() && i > 0; --i) {
        positionPub_.publish(setpoint);
        ros::spinOnce();
        loopRate.sleep();
      }
    }
    ROS_INFO("\033[33m ---> Ready to set %s mode !\033[0m", modeName.c_str());
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.custom_mode = modeName;
    if (modeClient_.call(srv_setMode) && srv_setMode.response.mode_sent){
      ROS_INFO("\033[32m ---> Set %s Mode Success !\033[0m", modeName.c_str());
      return true;
    }
    ROS_INFO("\033[31m ---> Set %s Mode Fail !\033[0m", modeName.c_str());
    return false;
  }

  void setPoint(geometry_msgs::PoseStamped& point){
    if(controlFSM_ != ActionFSM::Position){
      controlFSM_ = ActionFSM::Position;
    }
    msgPosition_ = point;
  }

  void setVel(geometry_msgs::TwistStamped& vel){
    if(controlFSM_ != ActionFSM::Velocity){
      controlFSM_ = ActionFSM::Velocity;
    }
    msgVel_ = vel;
  }

  void setPose(mavros_msgs::AttitudeTarget& pose){
    if(controlFSM_ != ActionFSM::Pose){
      controlFSM_ = ActionFSM::Pose;
    }
    msgPose_ = pose;
  }

  void setPVAY(mavros_msgs::PositionTarget& pvay){
    if(controlFSM_ != ActionFSM::PVAY){
      controlFSM_ = ActionFSM::PVAY;
    }
    msgPVAY_ = pvay;
  }

  void moveBodyPoint(Eigen::Vector3d point){
    Eigen::Vector3d temp;
    double siny = sin(motionState_.rpy[2]);
    double cosy = cos(motionState_.rpy[2]);
    temp[0] = point[0] * cosy - point[1] * siny;
    temp[1] = point[0] * siny + point[1] * cosy;
    temp[2] = point[2];
    msgPosition_.pose.position.x = motionState_.pt[0] + temp[0];
    msgPosition_.pose.position.y = motionState_.pt[1] + temp[1];
    msgPosition_.pose.position.z = motionState_.pt[2] + temp[2];
    msgPosition_.pose.orientation = uavPose_.pose.pose.orientation;
    if(controlFSM_ != ActionFSM::Position){
      controlFSM_ = ActionFSM::Position;
    }
  }

  void moveBody(double x, double y, double z){
    Eigen::Vector3d temp;
    double siny = sin(motionState_.rpy[2]);
    double cosy = cos(motionState_.rpy[2]);
    temp[0] = x * cosy - y * siny;
    temp[1] = x * siny + y * cosy;
    temp[2] = z;
    msgPosition_.pose.position.x = motionState_.pt[0] + temp[0];
    msgPosition_.pose.position.y = motionState_.pt[1] + temp[1];
    msgPosition_.pose.position.z = motionState_.pt[2] + temp[2];
    msgPosition_.pose.orientation = uavPose_.pose.pose.orientation;
    if(controlFSM_ != ActionFSM::Position){
      controlFSM_ = ActionFSM::Position;
    }
  }

  void moveBodyVel(Eigen::Vector3d vel){
    Eigen::Vector3d temp;
    double siny = sin(motionState_.rpy[2]);
    double cosy = cos(motionState_.rpy[2]);
    temp[0] = vel[0] * cosy - vel[1] * siny;
    temp[1] = vel[0] * siny + vel[1] * cosy;
    temp[2] = vel[2];
    msgVel_.twist.linear.x = temp[0];
    msgVel_.twist.linear.y = temp[1];
    msgVel_.twist.linear.z = temp[2];
    if(controlFSM_ != ActionFSM::Velocity){
      controlFSM_ = ActionFSM::Velocity;
    }
  }

  void moveBodyVel(double x, double y, double z){
    Eigen::Vector3d temp;
    double siny = sin(motionState_.rpy[2]);
    double cosy = cos(motionState_.rpy[2]);
    temp[0] = x * cosy - y * siny;
    temp[1] = x * siny + y * cosy;
    temp[2] = z;
    msgVel_.twist.linear.x = temp[0];
    msgVel_.twist.linear.y = temp[1];
    msgVel_.twist.linear.z = temp[2];
    if(controlFSM_ != ActionFSM::Velocity){
      controlFSM_ = ActionFSM::Velocity;
    }
  }

  void rotateAngleto(double degree)
  {
    double setyaw = degree;
    msgPosition_.pose.position.x = motionState_.pt[0];
    msgPosition_.pose.position.y = motionState_.pt[1];
    msgPosition_.pose.position.z = motionState_.pt[2];
    msgPosition_.pose.orientation.x = 0;
    msgPosition_.pose.orientation.y = 0;
    msgPosition_.pose.orientation.z = sin(setyaw * M_PI / 360);
    msgPosition_.pose.orientation.w = cos(setyaw * M_PI / 360);
    if(controlFSM_ != ActionFSM::Position){
      controlFSM_ = ActionFSM::Position;
    }
  }

  State getPose(){
    return motionState_;
  }

  nav_msgs::Odometry getOdom(){
    return uavPose_;
  }

  // ---------- 实机飞行的特殊 API--------------
  void waitTakeoff(){
    ros::Rate loopRate = ros::Rate(10);
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = uavPose_.pose.pose.position.x;
    setpoint.pose.position.y = uavPose_.pose.pose.position.y;
    setpoint.pose.position.z = uavPose_.pose.pose.position.z;
    setPoint(setpoint);
    while(ros::ok()){
      ros::spinOnce();
      if(uavState_.mode == "OFFBOARD")
        return;
      loopRate.sleep();
    }
  }


public: 
  typedef std::unique_ptr<Actuator> Ptr;

private:
  ros::NodeHandle    nh_;
  ros::Subscriber    stateSub_;
  ros::Subscriber    odomSub_;
  ros::Subscriber    accSub_;
  ros::Subscriber    homeSub_;
  ros::Publisher     positionPub_;
  ros::Publisher     velPub_;
  ros::Publisher     pvayPub_;
  ros::Publisher     posePub_;
  ros::ServiceClient landClient_;
  ros::ServiceClient armingClient_;
  ros::ServiceClient modeClient_;
  mavros_msgs::State uavState_;
  nav_msgs::Odometry uavPose_;
  State motionState_;
  
  bool FLAG_homeSet_;
  bool FLAG_running_;
  ActionFSM controlFSM_;
  std::thread* threadPub_ = nullptr;
  
  geometry_msgs::PoseStamped  msgPosition_;
  geometry_msgs::TwistStamped msgVel_;
  mavros_msgs::AttitudeTarget msgPose_;
  mavros_msgs::PositionTarget msgPVAY_;
  
}; 
// Class Actuator END ========================================================