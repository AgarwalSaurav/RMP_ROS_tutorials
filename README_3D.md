# 3D Environment and 6D Robot

Launch:
`roslaunch sphero6d_bringup rviz_trajectory.launch`


## Packages and Files

#### Visualization using rviz

1. `visualizations/rviz_environment3d`
Load 3D objects into rviz. The data file is located in `data/environment.yaml`

2. `visualizations/rviz_path3d`

   Visualization of a given path in 3d. The input file is in sub-folder `data`.

   `data/path` is a list of 3D coordinates.

   Format: `<x_coordinate y_coordinate z_coordinate>`
   
3. `visualizations/rviz_graph3d`
   Visualization of graph. The input files are in sub-folder `data`.

   a. `data/vertex_data` is a vertex list.

   Format: `<unique_vertex_id x_coordinate y_coordinate z_coordinate>`

   `data/edge_data` is an edge list.

   Format: `<vertex_id vertex_id>`

#### Sphero6d (6DoF robot)

1. `sphero6d/sphero6d_description`

   `urdf` folder contains URDF file for the robot

   `config` folder provides configuration for the robot

2. `sphero6d/sphero6d_bringup`

   `launch` folder contains launch files for the robot

   `rviz.launch` launches just the robot

   `rviz_trajectory.launch` launch the robot with the trajectory node and moves the robot through a sequence of configurations. Uncomment graph3d node to visualize graph 3d.

3. `sphero6d/sphero6d_operations`

   `scripts/trajectory_publisher_list.py`: Python script file for sending a list of configurations to robot using a file. The script translates the robot in all three directions and rotates only in yaw direction.

   `data/configurations`: List of configurations (x, y, z, yaw) for the robot to follow

   Optional:

   `scripts/trajectory_publisher.py`: Python script file to move robot from start configuration to goal configuration. We need to modify the file to get the robot follow a particular path. Constant velocity trajectory and cubic trajectory are already provided.
