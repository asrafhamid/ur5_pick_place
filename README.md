# ur5_pick_place

## Overview

UR5 simulation performing pick and pick operation. Script adapted from move_group_python_interface tutorial. 
- Reads in object pose from [Obj Detection](https://github.com/aychaplin/ur5_pick_place)
- Pick and place operation using moveit interface [Move Group Python](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html)

**Keywords:** universal robot, pick and place, ur5, moveit


**Author: Mohd Asraf <br />

The ur5_pick_place package has been tested under [ROS] Melodic on Ubuntu 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


![Example image](doc/example.jpg)


## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Universal Robot](https://github.com/ros-industrial/universal_robot) (ROS-Industrial Universal Robot meta-package)
- [Obj Detection](https://github.com/aychaplin/ur5_pick_place) (opencv ros-wrapper for object tracking)
- [Realsense Gazebo plugin](https://github.com/pal-robotics/realsense_gazebo_plugin) (Gazebo ROS plugin for the Intel D435 realsense camera)


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/aychaplin/ur5_pick_place.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

Source gazebo model

	source gazebo model
	export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/usr/share/gazebo-9/models:[your_workspace_directory]/src/ur5_pick_place/models"

## Usage


For starting gazebo simulation run:

	roslaunch ur5_pick_place ur5.launch
  
For setting up the MoveIt! nodes to allow motion planning run:

	roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
  
For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

	roslaunch ur5_moveit_config moveit_rviz.launch config:=true
  
Run pick_place node with:

	rosrun ur5_pick_place ur5_pick_place.py


## Launch files

* **ur5.launch:** simulation with gazebo

     - **`limited`** limits joint range [-PI, PI] on all joints. Default: `true`.
     - **`gui`** Starts gazebo gui. Default: `true`.

## Nodes

### ur_pick_place

Reads object pose, picks up object and place at defined postion.


#### Subscribed Topics

* **`/camera/object_track`** ([geometry_msgs/PoseStamped])

	The object pose for arm to pick up.




## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[geometry_msgs/PoseStamped]: http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html
