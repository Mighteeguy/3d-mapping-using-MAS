# 3-D Mapping using Multi-Agent Systems

Unmanned Aerial Vehicles (UAVs) autonomously explore and map an unknown environment using Frontier Exploration Algorithm and the Octomap Library. This repository uses the fundamentals of the [Caltech Samaritan](https://github.com/TimboKZ/caltech_samaritan/blob/master/README.md) and [Hector Quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) Repositories.


## Installation

This project was developed for ROS Melodic (Ubuntu 18.04). 
Run the following command on your terminal to install the necessary dependancies:

```bash
sudo apt-get install ros-melodic-octomap ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins ros-melodic-navigation ros-melodic-teb-local-planner ros-melodic-ros-numpy ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-unique-identifier ros-melodic-geographic-info ros-melodic-laser-geometry ros-melodic-tf-conversions ros-melodic-tf2-geometry-msgs ros-melodic-joy
```

Create a [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), and clone and compile this package (using ssh here):
```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:Mighteeguy/3d-mapping-using-MAS.git
cd ../ 
catkin_make
source devel/setup.bash
```

## Running the simulation 
### 3-D mapping using 2 UAVs (Quick start)
1. Initialize Gazebo with a sample 4-room environment using: 
   ```
   roslaunch caltech_samaritan full_indoors.launch
   ```
2. Start each UAV's exploration scripts in different terminals as: 

   ```
   source devel/setup.bash && rosrun caltech_samaritan start_exploration.py uav1 2
   ```

   ```
   source devel/setup.bash && rosrun caltech_samaritan start_exploration.py uav2 2
   ```

### 3-D mapping using multiple UAVs
To run the simulation with an increased (or decreased) number of UAVs,
1. Ensure that the code for only those UAVs that will be spawned is uncommented in the following two files: 1. [full_indoors.launch](caltech_samaritan/launch/full_indoors.launch), and 2. [spawn_two_quadrotors.launch](hector_quadrotor/hector_quadrotor_gazebo/launch/spawn_two_quadrotors.launch).
2. Initialize Gazebo with a sample 4-room environment using: 
   ```
   roslaunch caltech_samaritan full_indoors.launch
   ```
   The environment can be changed from the [full_indoors.launch](caltech_samaritan/launch/full_indoors.launch) file. The 3 main ones are a 4-room, 3-room, and a 2-room environment. 

3. Start each UAV's exploration scripts in new terminals as: _rosrun caltech\_samaritan start\_exploration.py uav[digit] [number of UAVs]_.

   E.g.: To start exploration scripts of UAV1, UAV2, and UAV3, run the following commands in different terminals:
   
   ```
   source devel/setup.bash && rosrun caltech_samaritan start_exploration.py uav1 3
   ```

   ```
   source devel/setup.bash && rosrun caltech_samaritan start_exploration.py uav2 3
   ```

   ```
   source devel/setup.bash && rosrun caltech_samaritan start_exploration.py uav3 3
   ```

## Demo
3-D mapping using 4 UAVs

<video src='https://user-images.githubusercontent.com/43675847/180644628-e7be5256-4254-4283-82dd-4a96f02d983d.mp4' width=180 />
