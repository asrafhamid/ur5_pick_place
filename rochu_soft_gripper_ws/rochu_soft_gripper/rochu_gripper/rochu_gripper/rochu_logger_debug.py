# /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import Log
import csv
import ros2pkg
from rclpy.time import Time
import time
from rosidl_runtime_py import get_interface_path
# from ament_index_python.packages import get_package_directory
import os
from pathlib import Path


class LogNode(Node):
    def __init__(self):
        super().__init__('rochu_logger_node')

        default_file_path =  str(Path.home())+"/logs/output.csv"
        self.declare_parameter("file_path",default_file_path)
        self.file_path = self.get_parameter('file_path')._value

        if not os.path.isdir(os.path.dirname(os.path.realpath(self.file_path))):
            os.mkdir(os.path.dirname(os.path.realpath(self.file_path)))
        
        self.log_sub = self.create_subscription(Log,'/rosout',self.log_sub,10)
        self.start_time =  time.time()
        self.csvfile =  open(self.file_path, 'w', newline='')
        fieldnames = ['debug_level', 'msg',"time"]
        self.writer = csv.DictWriter(self.csvfile, fieldnames=fieldnames)

    def log_sub(self,msg):
        # print(msg)
        if msg.name == "rochu_gripper_node" and msg.level >= 20:
            self.writer.writerow({'debug_level': msg.level, 'msg': msg.msg,'time': time.time()-self.start_time})
    

def main(args=None):
    rclpy.init(args=args)
    log_node= LogNode()
    try:
         rclpy.spin(log_node)

    except KeyboardInterrupt:
        pass

    finally:
        log_node.csvfile.close()
        log_node.destroy_node()
        # rclpy.shutdown()

    
    rclpy.shutdown()    
    
if __name__ == '__main__':
    main()