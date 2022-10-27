#include <runnable/competition.h>
#include <runnable/simulate.h>
#include <ros/ros.h>


using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autofly_node");
  Simulate::run2();
}