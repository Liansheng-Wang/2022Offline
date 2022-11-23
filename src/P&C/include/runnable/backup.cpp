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