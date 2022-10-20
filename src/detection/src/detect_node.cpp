#include <thread>
#include <detection.hpp>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect"); 

  ros::NodeHandle nh;               

  Detector detector(nh);

  std::thread DetectorThread(&Detector::runThread, &detector);
  
  DetectorThread.join();
  
  return 0;
}