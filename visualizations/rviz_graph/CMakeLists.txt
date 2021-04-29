cmake_minimum_required(VERSION 3.0.2)
project(rviz_graph)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rviz_graph
#  CATKIN_DEPENDS geometry_msgs roscpp visualization_msgs
#  DEPENDS system_lib
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(draw_graph src/draw_graph.cpp)
target_link_libraries(draw_graph ${catkin_LIBRARIES})
