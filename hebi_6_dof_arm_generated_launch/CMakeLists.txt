cmake_minimum_required(VERSION 2.8.3)
project(hebi_6_dof_arm_generated_launch)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # hebiros  ## Uncomment once hebiros node is on ROS Kinetic build farm
  roscpp
  rospy
  gazebo_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
