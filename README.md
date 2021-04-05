# RMP_holonomic_sphero

The library provides ROS nodes and launch files for a planar holonomic 3DoF robot. The library is designed for testing out simple Robot Motion Planning algorithms. Currently, the library does not consider dynamics of the robot.

The goal of this repository to provide code base that students can use to visualize algorithms and build their own ROS packages.

## Quick Start

1. Clone the repository into your workspace
   `git clone git@github.com:AgarwalSaurav/RMP_holonomic_sphero.git`
2. Run `catkin_make` on the top level directory of your workspace
3. `source devel/setup.bash`
4. Launch:
   `roslaunch sphero_bringup rviz_trajectory.launch`

You will notice that the robot is moving following a cubic trajectory.



## Packages and Files

1. `sphero_description`
   `urdf` folder contains URDF file for the robot
   `config` folder provides configuration for the robot

2. `sphero_bringup`

   `launch` folder contains launch files for the robot

   `rviz.launch` launches just the robot

   `rviz_trajectory.launch` launch the robot with the trajectory node (see below)

3. `sphero_operations`
   `scripts/trajectory_publisher.py`: Python script file for sending position coordinates to robot
   `src/trajectory_publisher.cpp`: C++ file for sending position coordinates to robot