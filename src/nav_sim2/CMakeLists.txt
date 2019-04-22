cmake_minimum_required(VERSION 2.8.3)
project(nav_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
