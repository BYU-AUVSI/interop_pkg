cmake_minimum_required(VERSION 2.8.3)
project(interop_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosplane_msgs
  std_msgs
  uav_msgs
)

catkin_package(
  CATKIN_DEPENDS rosplane_msgs std_msgs uav_msgs
)

