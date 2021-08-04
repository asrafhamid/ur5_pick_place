## Setup docker container
1. Install docker and setup the permission. You can refer to this [link](https://www.digitalocean.com/community/questions/how-to-fix-docker-got-permission-denied-while-trying-to-connect-to-the-docker-daemon-socket).

/home/hopermf/asraf_ws/src/ur5_pick_place/rochu_soft_gripper_ws


- Build docker image
```docker build -t rochu .```

- Run docker image:
```docker run -it --net=host rochu```
```tmux if needed```

## Control the gripper via ROS2 message
### Terminal 1:
```
ros2 run rochu_gripper rochu_gripper_node 
```

### Terminal 2:
1. Grab mode \
```
ros2 topic pub --once /rochu/request rochu_gripper_msgs/msg/GripperRequest "{name: "1", request_mode: {value: 0}, effort: 50}"
```
2. Release mode \
```
ros2 topic pub --once /rochu/request rochu_gripper_msgs/msg/GripperRequest "{name: "1", request_mode: {value: 2}, effort: 0}"
```
3. Idle mode \
```
ros2 topic pub --once /rochu/request rochu_gripper_msgs/msg/GripperRequest "{name: "1", request_mode: {value: 1}, effort: 0}"
```

## Control the gripper with ROS1 message and ros1_bridge
source install/setup.bash - ros2 ws

1. Check whether the rochu gripper message has been built successfully with the ros1_bridge \
```
ros2 run ros1_bridge dynamic_bridge --print-pairs |grep rochu
```

### Terminal 1:
```
ros2 run rochu_gripper rochu_gripper_node 
```

### Terminal 2:
```
ros2 run ros1_bridge dynamic_bridge
``` 

(make sure you have run roscore)\
 ```
roscore
```

### Terminal 3:
source devel/setup.bash - ros1_msg (cd to this workspace)

```
source ros1 workspace setup.bash
```

1. Grab mode \
```
rostopic pub --once /rochu/request rochu_gripper_msgs/GripperRequest "name: '1'
request_mode:
  value: 0
effort: 100" 
```
2. Release mode \

```
rostopic pub --once /rochu/request rochu_gripper_msgs/GripperRequest "name: '1'
request_mode:
  value: 2
effort: 0" 
```

3. Idle mode \
```
rostopic pub --once /rochu/request rochu_gripper_msgs/GripperRequest "name: '1'
request_mode:
  value: 1
effort: 100" 
```

