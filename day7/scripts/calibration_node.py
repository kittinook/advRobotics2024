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
        # self.sensor_subscription = self.create_subscription(Float64MultiArray, '/sensor_data/acc', self.sensor_callback, 10)
        self.sensor_subscription = self.create_subscription(Float64MultiArray, '/sensor_data/gyro', self.sensor_callback, 10)
        self.create_timer( 0.01 , self.timer_callback)
        self.i = 0
        self.max_data = 1000
        self.isCalibrate = False
        self.collected_data = []  
        self.result = {}
    
    def timer_callback(self):
        if self.isCalibrate == False and self.i >= self.max_data:

            data_array = np.array( self.collected_data )
            self.result["mean"] = np.mean(self.collected_data,0).tolist()
            cov = np.cov(data_array.T)
            self.result["covariance"] = np.reshape(cov, (cov.shape[0] * cov.shape[1])).tolist()
            calibration_path = get_package_share_directory('calibration')
            file = os.path.join(calibration_path, 'config', 'sensor_properties.yaml')
            # with open(file,'w') as f:
            #     yaml.dump( {'mean': self.result["mean"], 'covariance': self.result["covariance"]},f )
            # os.system("gedit " + file)
            print("mean")
            print(self.result["mean"])
            print("covariance")
            print(self.result["covariance"])
            self.isCalibrate = True

    def sensor_callback(self,msg):
        # self.action_server.current_data = msg.data
        if self.i < self.max_data:
            print("collected data : ", self.i)
            self.collected_data.append(msg.data)
            self.i = self.i + 1

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
