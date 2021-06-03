# ur5_pick_place

## Overview

UR5 simulation performing pick and pick operation. Script adapted from move_group_python_interface tutorial. 
- Reads in object pose from [Obj Detection](https://github.com/aychaplin/obj_detection)
- Pick and place operation using moveit interface [Move Group Python](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html)

**Keywords:** universal robot, pick and place, ur5, moveit


**Author: Mohd Asraf <br />

The ur5_pick_place package has been tested under [ROS] Melodic, Noetic on Ubuntu 18.04 and Ubuntu 20.04 respectively.


![Example image](doc/example.jpg)

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Universal Robot](https://github.com/ros-industrial/universal_robot) (ROS-Industrial Universal Robot meta-package)
- [Obj Detection](https://github.com/aychaplin/obj_detection) (opencv ros-wrapper for object tracking)
- [Realsense Gazebo plugin](https://github.com/pal-robotics/realsense_gazebo_plugin) (Gazebo ROS plugin for the Intel D435 realsense camera)


#### Building/Installation

1. Source gazebo model path, easier to add to .bashrc
```
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/usr/share/gazebo-9/models:[your_workspace_directory]/src/ur5_pick_place/models"
```
2. To build from source, clone the latest version from this repository into your catkin workspace and compile the package using:
```
cd catkin_workspace/src
git clone https://github.com/aychaplin/ur5_pick_place.git
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
```

## Usage
1. For starting gazebo simulation run:
	```
	roslaunch ur5_pick_place ur5.launch
	```
2. For setting up the MoveIt! nodes to allow motion planning run:
	```
	roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
	``` 
3. For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
	```
	roslaunch ur5_moveit_config moveit_rviz.launch config:=true
	```
4. To start the above 3 in one launch (gazebo simulation, MoveIt! node and MoveIt! Motion Planning plugin):
```
roslaunch ur5_pick_place ur5_pick_place.launch
```
5. Run pick_place node with:
```
rosrun ur5_pick_place ur5_pick_place.py
```
6. Pick up the **red** coloured object by:
```
rostopic pub object_colour std_msgs/String "data: 'blue'" 
```

## Launch files

* **ur5.launch:** simulation with gazebo

     - **`limited`** limits joint range [-PI, PI] on all joints. Default: `true`.
     - **`gui`** Starts gazebo gui. Default: `true`.

* **ur5_pick_place.launch:** simulation with gazebo

     - **`limited`** limits joint range [-PI, PI] on all joints. Default: `true`.
     - **`gui`** Starts gazebo gui. Default: `true`.

## Nodes

### ur_pick_place

Reads object pose, picks up object and place at defined postion.


#### Subscribed Topics

* **`/camera/object_track`** ([geometry_msgs/PoseStamped])

	The object pose for arm to pick up.




## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/aychaplin/ur5_pick_place/issues).
