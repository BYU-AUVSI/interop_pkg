cmake_minimum_required(VERSION 2.8.3)
project(interop_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosflight_msgs
  std_msgs
  uav_msgs
)

catkin_package(
  CATKIN_DEPENDS rosflight_msgs std_msgs uav_msgs
)

