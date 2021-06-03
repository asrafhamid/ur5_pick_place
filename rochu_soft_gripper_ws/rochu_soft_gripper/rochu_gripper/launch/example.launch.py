from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import pathlib
import os 

def generate_launch_description():

    parameters_file_path = os.path.dirname(os.path.realpath(__file__))
    parameters_file_path += '/params.yaml'
    print(parameters_file_path)
    hard= '/home/waihong/ros2_ws/src/rochu_soft_gripper/rochu_gripper/config/params.yaml'
    return LaunchDescription([
    Node(
        package='rochu_gripper',
        node_executable='rochu_gripper_node',
        name='rochu_gripper',
        parameters = [parameters_file_path],
    )
    ,Node(
        package='rochu_gripper',
        node_executable='rochu_logger_node',
        name='rochu_logger',
    )
    ])
    