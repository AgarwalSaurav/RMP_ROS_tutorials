cmake_minimum_required(VERSION 3.0.2)
project(sphero_operations)

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

catkin_package()

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

add_executable(trajectory_publisher_cpp src/trajectory_publisher.cpp)
target_link_libraries(trajectory_publisher_cpp ${catkin_LIBRARIES})
