#pragma once

#include <param.h>
#include <common.h>
#include <planner.h>
#include <uav_control/controller.h>
#include <vector>
#include <Eigen/Core>

namespace Simulate{
  std::vector<Eigen::Vector3d> waypoints;
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
  }

  void run()
  {
    InitWayPoints();
    ros::NodeHandle nh;
    auto positionPub_ = nh.advertise<geometry_msgs::Pose>("/test/points", 10);
    State curState;
    State tarState;
    Planner planner;
    planner.setTargetMarker(waypoints, poses);
    Controller controller;
    controller.waitHomeSet();
    controller.arm();
    controller.setMode("OFFBOARD");
    controller.takeoff(1.1);
    ros::spinOnce();
    curState = controller.getPose();
    tarState.pt  = waypoints[0];
    tarState.vel <<1, 0, 0;
    planner.plan(curState, tarState);
    double t_t = planner.getTotalTime();
    geometry_msgs::TwistStamped cmdVel;
    double rr_t = 0;
    ros::Time startTime  = ros::Time::now();
    ros::Time actionTime = ros::Time::now();
    while(ros::ok() && rr_t < t_t)
    {
      ros::spinOnce();
      actionTime = ros::Time::now();
      rr_t = (actionTime - startTime).toSec();
      auto pt2follow =  planner.getPathPoint(rr_t);
      curState = controller.getPose();
      Eigen::Vector3d velEigen = 1.0 *(pt2follow.pt - curState.pt) + pt2follow.vel - 0.2 * (curState.vel - pt2follow.vel);
      cmdVel.header.stamp = ros::Time::now();
      cmdVel.twist.linear.x = velEigen[0];
      cmdVel.twist.linear.y = velEigen[1];
      cmdVel.twist.linear.z = velEigen[2];
      controller.setVel(cmdVel);
    }
    controller.land();
  }

  // 测试全状态输出
  void run2()
  {
    InitWayPoints();
    ros::NodeHandle nh;
    auto positionPub_ = nh.advertise<geometry_msgs::Pose>("/test/points", 10);
    State curState;
    State tarState;
    Planner planner;
    planner.setTargetMarker(waypoints, poses);
    Controller controller;
    controller.waitHomeSet();
    controller.arm();
    controller.setMode("OFFBOARD");
    controller.takeoff(1.1);

    // ====================================
    int index = -1;
    mavros_msgs::PositionTarget cmdPVAY;
    double rr_t, t_t;
    ros::Time startTime;
    ros::Time actionTime;

    for(int i = 0; i < 11; i++)
    {
      ros::spinOnce();
      index++;
      curState = controller.getPose();
      tarState.pt  = waypoints[index];
      tarState.vel = {UAVparam::MaxVel * cos(poses[index]/180*M_PI), UAVparam::MaxVel * sin(poses[index]/180*M_PI), 0};
      planner.plan(curState, tarState);
      t_t = planner.getTotalTime();
      rr_t = 0;
      startTime  = ros::Time::now();
      actionTime = ros::Time::now();
      while(ros::ok() && rr_t < t_t)
      {
        ros::spinOnce();
        actionTime = ros::Time::now();
        rr_t = (actionTime - startTime).toSec();
        State pt2follow =  planner.getPathPoint(rr_t);
        curState = controller.getPose();
        cmdPVAY.header.frame_id = cmdPVAY.FRAME_LOCAL_NED;
        cmdPVAY.coordinate_frame = 1;
        cmdPVAY.header.stamp = ros::Time::now();
        cmdPVAY.position.x = pt2follow.pt[0];
        cmdPVAY.position.y = pt2follow.pt[1];
        cmdPVAY.position.z = pt2follow.pt[2];
        cmdPVAY.velocity.x = pt2follow.vel[0];
        cmdPVAY.velocity.y = pt2follow.vel[1];
        cmdPVAY.velocity.z = pt2follow.vel[2];
        cmdPVAY.acceleration_or_force.x = pt2follow.acc[0];
        cmdPVAY.acceleration_or_force.y = pt2follow.acc[1];
        cmdPVAY.acceleration_or_force.z = pt2follow.acc[2];
        cmdPVAY.yaw = atan2(pt2follow.vel[1], pt2follow.vel[0]);
        controller.setPVAY(cmdPVAY);
      }
    }
    controller.land();
  }

};