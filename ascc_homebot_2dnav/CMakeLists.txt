cmake_minimum_required(VERSION 2.8.3)
project(ascc_homebot_2dnav)

find_package(catkin REQUIRED COMPONENTS
  move_base
  rosaria
  tf_pioneer
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
