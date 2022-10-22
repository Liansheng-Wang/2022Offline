#pragma once

#include <common.h>
#include <planner.h>
#include <uav_control/controller.h>
#include <vector>
#include <Eigen/Core>

namespace Simulate{
  std::vector<Eigen::Vector3d> waypoints;

  void InitWayPoints()
  {
    waypoints.push_back({6.5, 0.4, 0.8});
    waypoints.push_back({13,  0.2, 0.8});
    waypoints.push_back({14,  7.5, 0.8});
    waypoints.push_back({8,   7.5, 0.8});
    waypoints.push_back({1,   7.5, 0.8});
    waypoints.push_back({1,2,3});
    waypoints.push_back({1,2,3});
    waypoints.push_back({1,2,3});
    waypoints.shrink_to_fit();
  }

  void run()
  {
    InitWayPoints();
    ros::NodeHandle nh;
    auto positionPub_ = nh.advertise<geometry_msgs::Pose>("/test/points", 10);
    State curState;
    State tarState;
    Planner planner;
    Controller controller;
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
      // test
      geometry_msgs::Pose msgPoint;
      msgPoint.position.x = pt2follow.pt[0];
      msgPoint.position.y = pt2follow.pt[1];
      msgPoint.position.z = pt2follow.pt[2];
      msgPoint.orientation.x = pt2follow.acc[0];
      msgPoint.orientation.y = pt2follow.acc[1];
      msgPoint.orientation.z = pt2follow.acc[2];
      positionPub_.publish(msgPoint);
      // test end

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
};