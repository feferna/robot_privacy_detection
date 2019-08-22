cmake_minimum_required(VERSION 2.8.3)
project(robot_talks)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sound_play
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
