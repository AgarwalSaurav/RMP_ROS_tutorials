cmake_minimum_required(VERSION 3.0.2)
project(rviz_path3d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rviz_path3d
#  CATKIN_DEPENDS geometry_msgs roscpp visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(draw_path3d src/draw_path3d.cpp)
target_link_libraries(draw_path3d ${catkin_LIBRARIES})
