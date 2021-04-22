# RMP_ROS_tutorials

The library provides ROS nodes and launch files for a planar holonomic 3DoF robot. The library is designed for testing out simple Robot Motion Planning algorithms. Currently, the library does not consider dynamics of the robot.

The goal of this repository to provide code base that students can use to visualize algorithms and build their own ROS packages.

## Quick Start

1. Clone the repository into your workspace (`catkin_ws/src`)
   
   `git clone https://github.com/AgarwalSaurav/RMP_ROS_tutorials`
   
2. Run `catkin_make` on the top level directory of your workspace

3. `source devel/setup.bash`

4. Launch:
   `roslaunch sphero_bringup rviz_trajectory.launch`

a. You will notice that the robot is moving and is following a constant velocity trajectory.

b. The black polygon represents the outer boundary.

c. The red polygons represent the obstacles.

d. The green polyline represents the path of the robot.

e. The robot is shown as a red sphere along with its axes.



## Packages and Files

#### Sphero (plan 3DoF holonomic robot)

1. `sphero/sphero_description`
   
   `urdf` folder contains URDF file for the robot

   `config` folder provides configuration for the robot
   
2. `sphero/sphero_bringup`

   `launch` folder contains launch files for the robot

   `rviz.launch` launches just the robot

   `rviz_trajectory.launch` launch the robot with the trajectory node (see below)

3. `sphero/sphero_operations`
   
   `scripts/trajectory_publisher.py`: Python script file for sending position coordinates to robot
   
   `src/trajectory_publisher.cpp`: C++ file for sending position coordinates to robot
   
   We need to modify either of the above files to get the robot follow a particular path. Constant velocity trajectory and cubic trajectory are already provided.

#### Visualization using rviz

1. `visualizations/rviz_graph`
   
2. Visualization of graph. The input files are in sub-folder `data`.

   a. `data/vertex_data` is vertex list.

   Format: `<unique_vertex_id x_coordinate y_coordinate>`

   `data/edge_data` is edge list.

   Format: `<vertex_id vertex_id>`

3. `visualizations/rviz_path`

   Visualization of a given path. The input file is in sub-folder `data`.

   `data/path` is a list of 2D coordinates.

   Format: `<x_coordinate y_coordinate>`

4. `visualizations/rviz_polygon_env`

   Visualization of a polygonal environment. The input file is in sub-folder `data`.

   `data/environment` is a list of polygons. Each polygon is a list of 2D points. The polygons are separated by a blank line. The first polygon is the outer boundary. The remaining polygons are obstacles.

   Format: `<x_coordinate y_coordinate>`

   
