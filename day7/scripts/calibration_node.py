#!/usr/bin/python3

from concurrent.futures import Executor
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from calibration_interfaces.action import Calibrate
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
import os

class Calibration(Node):
    def __init__(self):
        super().__init__('calibration_subscriber')
        pass
        
    def timer_callback(self):
        pass

    def sensor_callback(self,msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Calibration()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        # executor.add_node(action_server)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            # action_server.destroy_node()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()
