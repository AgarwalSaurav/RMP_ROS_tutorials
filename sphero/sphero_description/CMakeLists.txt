cmake_minimum_required(VERSION 3.0.2)
project(sphero_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joint_state_publisher
  std_msgs
  urdf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
