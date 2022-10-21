#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>

using namespace std;

const double Takeoff_Height_ = 1.0;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pos;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_node");
  ros::NodeHandle nh("~");
  ros::Subscriber    state_sub           = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber    plane_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, local_pos_cb, ros::TransportHints().tcpNoDelay());
  ros::Publisher     local_pos_pub       = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",    10);
  ros::Publisher     local_vel_pub       = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Publisher     pose_pub            = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",     10);
  ros::ServiceClient land_client         = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  ros::ServiceClient arming_client       = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client     = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  ros::Rate rate(20.0);

  mavros_msgs::SetMode srv_setMode;
  mavros_msgs::CommandBool srv_arming;
  geometry_msgs::PoseStamped pose;
  ros::Time last_request;

  // Step 1.1: 等待 飞控 与 mavros 建立连接
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    ROS_INFO_STREAM_THROTTLE(20,"Not Connected");
    rate.sleep();
  }
  ROS_INFO("\033[32m ---> Connected ! \033[0m");

  // Step 1.2: 初始化起飞高度及相关变量
  srv_setMode.request.custom_mode = "OFFBOARD";
  srv_arming.request.value = true;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = Takeoff_Height_;

  // Step2: 解锁无人机
  last_request = ros::Time::now();
  arming_client.call(srv_arming);
  while (ros::ok()){
    ros::spinOnce();
    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0))){
      if( arming_client.call(srv_arming) &&
          srv_arming.response.success){
          ROS_INFO("Vehicle armed");
      }
      last_request = ros::Time::now();
    }else break;
    local_pos_pub.publish(pose);
    rate.sleep();
  }
  ROS_INFO("\033[32m ---> Disarmd Success! \033[0m");

  // Step3: 将无人机切换到 offboard模式 。
  // 注意: 无人机在进入offboard模式后,必须有一定频率的控制指令,否则飞机坠毁!
  // 真机飞行时，一定不可以代码切换 offboard ！！！
  last_request = ros::Time::now();
  set_mode_client.call(srv_setMode);
  while (ros::ok()){
    ros::spinOnce();
    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){
      if( set_mode_client.call(srv_setMode) &&
          srv_setMode.response.mode_sent){
          ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else {
      set_mode_client.call(srv_setMode);
      break;
    }
    local_pos_pub.publish(pose);
    rate.sleep();
  }
  ROS_INFO("\033[32m ---> Set OFFBOARD Mode Success! \033[0m");

  // Step4: 真实飞行时，等待手动切换 OFFBOARD 模式，在此之前在指令 buffer 中先压入指令
  pose.pose.position.x = local_pos.pose.position.x;
  pose.pose.position.y = local_pos.pose.position.y;
  pose.pose.position.z = local_pos.pose.position.z + Takeoff_Height_;
  while(ros::ok() && current_state.mode != "OFFBOARD"){
      ros::spinOnce();
      local_pos_pub.publish(pose);
      ROS_INFO_STREAM_THROTTLE(20,"\033[33m ---> Waiting change to OFFBOARD \033[0m");
      rate.sleep();
  }
  ROS_INFO("\033[32m ---> Detected OFFBOARD MODE! \033[0m");

  // Step5: 给一个位置指令给飞机，飞机起飞。
  ROS_INFO("\033[32m ---> Takeoff... \033[0m");
  last_request = ros::Time::now();
  while (ros::ok() && ros::Time::now() - last_request < ros::Duration(20.0))
  {
      ros::spinOnce();
      local_pos_pub.publish(pose);
      rate.sleep();
  }

  // 接下来，进入到比赛控制指令中来，通过发布 位置 or 速度 or 姿态指令控制无人机
    


}