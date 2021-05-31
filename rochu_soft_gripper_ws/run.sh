# Initialize default arguments
COMMAND=["tmux",". install/setup.bash","ros2 run ros1_bridge dynamic_bridge --print-pairs |grep rochu"]  # what to run once inside the container
COMMAND="tmux"
echo "starting container..."
docker run  -it \
            rochu_soft_gripper \
            launch-followme.sh