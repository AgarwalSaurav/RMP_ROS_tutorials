cmake_minimum_required(VERSION 3.0.2)
project(sphero6d_operations)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sphero6d_operations
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/trajectory_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/trajectory_publisher_list.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
