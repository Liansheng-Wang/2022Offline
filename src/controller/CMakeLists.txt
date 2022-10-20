cmake_minimum_required(VERSION 3.0.2)
project(controller)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  mavros
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_node src/example.cpp)
target_link_libraries(example_node ${catkin_LIBRARIES})