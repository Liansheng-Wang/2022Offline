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
  message_filters
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL    REQUIRED)

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
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(auto_fly src/autoFly.cpp)
target_link_libraries(auto_fly ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})