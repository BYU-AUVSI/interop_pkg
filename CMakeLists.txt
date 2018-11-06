cmake_minimum_required(VERSION 2.8.3)
project(interop_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosflight_msgs
  rospy
  sniper_cam
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

