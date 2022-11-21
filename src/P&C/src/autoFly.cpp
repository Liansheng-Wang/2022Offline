
#include <iostream>
#include <runnable/competition.h>
#include <runnable/simulate.h>
#include <ros/ros.h>


using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autofly_node");
  ros::NodeHandle nh;
  int useSim;
  nh.param<int>("/env/use_sim", useSim, 1);
  if(useSim == 0){
    std::cout << "\033[32m ---> Competition Running!" << std::endl;
    Competition::run(nh);
  }
  else if(useSim == 1){
    std::cout << "\033[32m ---> Simulate Running!" << std::endl;
    Simulate::run();
  }
  else{
    ROS_ERROR("Choose a errror param for env");
  }
  return 0;
}