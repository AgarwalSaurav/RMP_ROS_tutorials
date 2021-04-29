cmake_minimum_required(VERSION 3.0.2)
project(sphero6d_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joint_state_publisher
  std_msgs
  urdf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sphero6d_description
#  CATKIN_DEPENDS geometry_msgs joint_state_publisher std_msgs urdf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
