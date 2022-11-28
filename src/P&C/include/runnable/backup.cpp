// std::vector<Eigen::Vector3d> localwps;
// std::vector<double> localPoses;
// while(ros::ok()){
//   ros::spinOnce();
//   localwps.clear();
//   localPoses.clear();
//   uavPose = actuator.getPose();
//   State localtarget = planner.getLocalTarget(uavPose);
//   std::cout << "localtarget:  " << localtarget.pt.transpose() << "    " << planner.globalTraj_.last_progress_time_ << std::endl;
//   // step1. 规划局部轨迹。
//   if(isFree[MissionIndex_]){
//     // 选择局部估计的路点和pose
//     if(Flag_Detect_){
//       transDetect();
//       double dis1 = (localtarget.pt - uavPose.pt).norm();
//       double dis2 = (GlobalDetct_ - uavPose.pt).norm();
//       // order 一下两个点的位置
//       if(dis1 > dis2){
//         localwps.push_back(GlobalDetct_);
//         localwps.push_back(localtarget.pt);
//         localPoses.push_back(poses[MissionIndex_]);
//         localPoses.push_back(-404);
//       }else{
//         localwps.push_back(localtarget.pt);
//         localwps.push_back(GlobalDetct_);
//         localPoses.push_back(-404);
//         localPoses.push_back(poses[MissionIndex_]);
//       }
//     }else{
//       double dis1 = (localtarget.pt - uavPose.pt).norm();
//       double dis2 = (waypoints_[MissionIndex_] - uavPose.pt).norm();
//       // order 一下两个点的位置
//       if(dis1 > dis2){
//         localwps.push_back(waypoints_[MissionIndex_]);
//         localwps.push_back(localtarget.pt);
//         localPoses.push_back(poses[MissionIndex_]);
//         localPoses.push_back(-404);
//         endState = localtarget;
//       }else{
//         localwps.push_back(localtarget.pt);
//         localwps.push_back(waypoints_[MissionIndex_]);
//         localPoses.push_back(-404);
//         localPoses.push_back(poses[MissionIndex_]);
//         endState.pt = waypoints_[MissionIndex_];
//         endState.vel << UP::MaxVel * sin(poses[MissionIndex_] / 180 * M_PI), UP::MaxVel * cos(poses[MissionIndex_] / 180 * M_PI), 0;
//         endState.acc = Eigen::Vector3d::Zero();
//       }
//     }

//     planner.planLocalTraj(uavPose, endState, localwps, localPoses);
//     visualtool.setLocalTrj(planner.localTraj_);
//   }else{
//     // 触发避障的问题
    

//   }
//   std::cout << "  执行轨迹： " << std::endl;
//   while(ros::ok()){
//     // 规划完局部轨迹之后：
//     ros::spinOnce();
//     uavPose = actuator.getPose();
//     State pt2follow = planner.getLocalPathPoint(ros::Time::now().toSec());
//     cmdPVAY.header.frame_id = cmdPVAY.FRAME_LOCAL_NED;
//     cmdPVAY.coordinate_frame = 1;
//     cmdPVAY.header.stamp = ros::Time::now();
//     cmdPVAY.position.x = pt2follow.pt[0];
//     cmdPVAY.position.y = pt2follow.pt[1];
//     cmdPVAY.position.z = pt2follow.pt[2];
//     cmdPVAY.yaw = atan2(pt2follow.vel[1], pt2follow.vel[0]);
//     actuator.setPVAY(cmdPVAY);

//     if(isCross(uavPose)){
//       std::cout <<"Crossed: " << MissionNums_ << std::endl;
//       MissionNums_++;
//       break;
//     }
//     loopRate.sleep();
//   }

//   loopRate.sleep();
// }

// poses_ 中小于 -200 的是不约束，该点速度方向的点
waypoints_.push_back({3.0,    0.0,  1.5}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);  
waypoints_.push_back({4.0,    0.0,  1.5}); poses_.push_back(0);     isFree.push_back(true);  types.push_back(0);  // 1 号环         1
waypoints_.push_back({4.3,    0.0,  1.5}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);
waypoints_.push_back({8.0,   -0.4,  1.8}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4); 
waypoints_.push_back({9.0,   -0.4,  1.8}); poses_.push_back(0);     isFree.push_back(true);  types.push_back(0);  // 2 号环         4
waypoints_.push_back({13.0,  -1.08, 1.4}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);
waypoints_.push_back({14.285,-1.18, 1.2}); poses_.push_back(0);     isFree.push_back(true);  types.push_back(2);  // 第一个墙面      6
waypoints_.push_back({14.585,-1.18, 1.2}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);
waypoints_.push_back({20.8,  -1.18, 1.9}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);
waypoints_.push_back({22.8,  -0.8,  1.5}); poses_.push_back(10);    isFree.push_back(false); types.push_back(0);  // 11 号环        9
waypoints_.push_back({27.50,  1.0,  1.5}); poses_.push_back(60);    isFree.push_back(true);  types.push_back(0);  // 这个type = 0, 是为了能够检测出来12  10             
// waypoints_.push_back({24.4,   2.6,  1.5}); poses_.push_back(90);  isFree.push_back(true); types.push_back(0);   // 12-1 号环     11 
waypoints_.push_back({27.90,  2.8,  1.5}); poses_.push_back(90);    isFree.push_back(true);  types.push_back(0);  // 12-2 号环      11
waypoints_.push_back({22.80,  6.54, 1.5}); poses_.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 13 号环        12
waypoints_.push_back({17.72,  6.05, 1.35});poses_.push_back(-404);  isFree.push_back(false); types.push_back(2);  // 插补一个避障的点 13
waypoints_.push_back({14.285, 5.94, 1.2}); poses_.push_back(180);   isFree.push_back(false); types.push_back(2);  // 第二个墙面      14
waypoints_.push_back({9.0,    5.75, 1.5}); poses_.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 8 号环动态      15
waypoints_.push_back({3.9,    4.6,  1.5}); poses_.push_back(180);   isFree.push_back(true);  types.push_back(0);  // 9 号环         16 
waypoints_.push_back({-1.0,   4.6,  1.5}); poses_.push_back(180);   isFree.push_back(true);  types.push_back(3);  // 10 号异型环     17
waypoints_.push_back({-1.9,   4.6,  1.5}); poses_.push_back(-404);  isFree.push_back(true);  types.push_back(4);  // 降落点  