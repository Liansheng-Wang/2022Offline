#pragma once

#include <common/base.h>
#include <plan/planner.h>
#include <control/actuator.h>
#include <visual/visualTools.h>
#include <vector>
#include <Eigen/Core>

// #include <plan_env/voxelmap.h>   // FIXME: 该文件暂时有问题

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
    State curState, tarState;
    VisualTool visualtool;
    Planner planner;
    Actuator actuator;
    visualtool.setTargetMarker(waypoints, poses);
    actuator.waitHomeSet();
    actuator.arm();
    actuator.setMode("OFFBOARD");
    actuator.takeoff(1.1);

    // ====================================
    int index = -1;
    mavros_msgs::PositionTarget cmdPVAY;
    double rr_t, t_t;
    ros::Time startTime;
    ros::Time actionTime;

    ros::spinOnce();
    index++;
    curState = actuator.getPose();
    tarState.pt  = waypoints[index];
    tarState.vel = {UAVparam::MaxVel * cos(poses[index]/180*M_PI), UAVparam::MaxVel * sin(poses[index]/180*M_PI), 0};
    planner.planGlobalTraj(curState, tarState, waypoints, poses);
    visualtool.setGlobalTrj(planner.globalTraj_);
    t_t = planner.getGlobalTotalTime();
    rr_t = 0;
    startTime  = ros::Time::now();
    actionTime = ros::Time::now();
    while(ros::ok() && rr_t < t_t)
    {
      ros::spinOnce();
      actionTime = ros::Time::now();
      rr_t = (actionTime - startTime).toSec();
      // FIXME: getGlobalPathPoint() 这个函数肯定是有问题的。
      State pt2follow =  planner.getGlobalPathPoint(actionTime.toSec());  
      curState = actuator.getPose();
      cmdPVAY.header.frame_id = cmdPVAY.FRAME_LOCAL_NED;
      cmdPVAY.coordinate_frame = 1;
      cmdPVAY.header.stamp = ros::Time::now();
      cmdPVAY.position.x = pt2follow.pt[0];
      cmdPVAY.position.y = pt2follow.pt[1];
      cmdPVAY.position.z = pt2follow.pt[2];
      // 下面的指令不用给
      // cmdPVAY.velocity.x = pt2follow.vel[0];
      // cmdPVAY.velocity.y = pt2follow.vel[1];
      // cmdPVAY.velocity.z = pt2follow.vel[2];
      // cmdPVAY.acceleration_or_force.x = pt2follow.acc[0];
      // cmdPVAY.acceleration_or_force.y = pt2follow.acc[1];
      // cmdPVAY.acceleration_or_force.z = pt2follow.acc[2];
      cmdPVAY.yaw = atan2(pt2follow.vel[1], pt2follow.vel[0]);
      actuator.setPVAY(cmdPVAY);
    }
    std::cout << "end!!!!!" << std::endl; 
    actuator.land();

    ros::Rate loopRate(5);
    while(ros::ok())
    {
      ros::spinOnce();
      loopRate.sleep();
    }
  }

};