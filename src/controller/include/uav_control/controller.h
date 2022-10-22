#pragma once;

#include <string>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>


enum ControlFSM
{
  Init,
  Pose,
  Position,
  Velocity
};


class Controller
{
public:
  Controller(): loopRate_(30), controlFSM_(ControlFSM::Init), FLAG_running_(true)
  {
    stateSub_           = nh_.subscribe<mavros_msgs::State>("/mavros/state", 1, &Controller::StateCb, this, ros::TransportHints().tcpNoDelay());
    poseSub_            = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &Controller::PoseCb, this, ros::TransportHints().tcpNoDelay());
    positionPub_        = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",    10);
    velPub_             = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    posePub_            = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",     10);
    landClient_         = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    armingClient_       = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    modeClient_         = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    threadPub_ = new std::thread(boost::bind(&Controller::publishThread, this));
  }

  ~Controller(){
    FLAG_running_ = false;
    threadPub_->join();
    delete threadPub_;
    ROS_INFO("\033[32m ---> Controller closed ^_^ ! \033[0m");
  }

  void StateCb(const mavros_msgs::State::ConstPtr& msg){
    uavState_ = *msg;
  }

  void PoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uavPose_ = *msg;
  }

  void publishThread()
  {
    while(ros::ok() && FLAG_running_)
    {
      switch (controlFSM_)
      {
      case Init:
      {
        break;
      }
      case Pose:
      {
        posePub_.publish(msgPose_);
        break;
      }
      case Position:
      {
        positionPub_.publish(msgPosition_);
        break;
      }
      case Velocity:
      {
        velPub_.publish(msgVel_);
        break;
      }      
      default:
        break;
      }
      loopRate_.sleep();
    }
  }

  void takeoff(double height){
    ros::spinOnce();
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = uavPose_.pose.position.x;
    setpoint.pose.position.y = uavPose_.pose.position.y;
    setpoint.pose.position.z = uavPose_.pose.position.z + height;
    msgPosition_ = setpoint;
    controlFSM_ = ControlFSM::Position;
    while(abs(uavPose_.pose.position.z - setpoint.pose.position.z) > 0.25){
      ros::spinOnce();
      loopRate_.sleep();
    }
    ROS_INFO("\033[32m ---> Take off success ! \033[0m");
  }

  bool land(){
    mavros_msgs::CommandTOL srvLand{};
    if (landClient_.call(srvLand) && srvLand.response.success)
      return true;
    else
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
    if(modeName == "OFFBOARD"){
      ros::spinOnce();
      geometry_msgs::PoseStamped setpoint;
      setpoint.pose.position.x = uavPose_.pose.position.x;
      setpoint.pose.position.y = uavPose_.pose.position.y;
      setpoint.pose.position.z = uavPose_.pose.position.z;
      msgPosition_ = setpoint;
      controlFSM_ = ControlFSM::Position;
      for (int i = 100; ros::ok() && i > 0; --i) {
        positionPub_.publish(setpoint);
        ros::spinOnce();
        loopRate_.sleep();
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

public:
  ros::NodeHandle    nh_;
  ros::Subscriber    stateSub_;
  ros::Subscriber    poseSub_;
  ros::Publisher     positionPub_;
  ros::Publisher     velPub_;
  ros::Publisher     posePub_;
  ros::ServiceClient landClient_;
  ros::ServiceClient armingClient_;
  ros::ServiceClient modeClient_;
  ros::Rate loopRate_;
  mavros_msgs::State uavState_;
  geometry_msgs::PoseStamped uavPose_;
  
  bool FLAG_running_;
  ControlFSM controlFSM_;
  std::thread* threadPub_ = nullptr;
  
  geometry_msgs::PoseStamped  msgPosition_;
  geometry_msgs::TwistStamped msgVel_;
  mavros_msgs::AttitudeTarget msgPose_;
  
}; 
// Class Controller END ========================================================