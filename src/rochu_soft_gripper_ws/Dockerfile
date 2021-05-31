FROM osrf/ros:eloquent-ros1-bridge

ENV HOME /home/rochu_soft_gripper_ws/
ENV ROS_DISTRO1 melodic
ENV ROS_DISTRO2 eloquent

# Install build tools
RUN apt-get update && apt-get install -y \
      python-catkin-tools python-rosdep tmux vim \
      python3-pip \
      python-rospkg \
    && rm -rf /var/lib/apt/lists/*

# ROS2 rochu soft gripper package
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO2}/setup.bash"
RUN mkdir -p /home/rochu_soft_gripper_ws
WORKDIR  /home/rochu_soft_gripper_ws/
COPY rochu_soft_gripper src
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO2}  -yr
RUN pip3 install pyModbusTCP
RUN /ros_entrypoint.sh \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE && \
    sed -i '$isource "/home/rochu_soft_gripper_ws/install/setup.bash"' /ros_entrypoint.sh

# ROS1 rochu msg ros1_bridge
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO1}/setup.bash"
RUN mkdir -p /home/ros1_bridge_ws/ros1_msg/src
WORKDIR  /home/ros1_bridge_ws/ros1_msg
COPY rochu_gripper_msgs src/rochu_gripper_msgs
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO1}  -yr
RUN catkin config --extend /opt/ros/$ROS_DISTRO1 && \
    catkin build -c -s

# ROS2 WS - ros1_bridge
RUN mkdir -p /home/ros1_bridge_ws/dev_ws/src
WORKDIR  /home/ros1_bridge_ws/dev_ws/src
COPY rochu_soft_gripper/rochu_gripper_msgs rochu_gripper_msgs
RUN git clone -b eloquent https://github.com/ros2/ros1_bridge.git
WORKDIR  /home/ros1_bridge_ws/dev_ws
RUN apt-get update \
  &&  apt-get install -y ros-eloquent-launch* \
  && apt-get clean

RUN bash -c "source /home/ros1_bridge_ws/ros1_msg/devel/setup.bash \
        && source /opt/ros/eloquent/setup.bash \
        && colcon build --symlink-install --packages-skip ros1_bridge \
        && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

# COPY launch-followme.sh launch-followme.sh 
# ENTRYPOINT ["tmux","./launch-followme.sh"]
