cmake_minimum_required(VERSION 3.0.2)
project(sphero_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  urdf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
