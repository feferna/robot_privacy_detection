cmake_minimum_required(VERSION 2.8.3)
project(privacy_controller)

find_package(catkin REQUIRED COMPONENTS
  ascc_homebot_2dnav
  pir_receiver_no_socket
  robot_talks
  roscpp
  rospy
  std_msgs
)

catkin_package()
