cmake_minimum_required(VERSION 3.0.2)
project(competition_sim)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  mavros
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
