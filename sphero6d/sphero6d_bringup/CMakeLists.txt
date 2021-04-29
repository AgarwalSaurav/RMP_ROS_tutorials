cmake_minimum_required(VERSION 3.0.2)
project(sphero6d_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sphero6d_bringup
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
