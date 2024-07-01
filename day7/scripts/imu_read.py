#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

class IMUSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.read_and_publish)
        self.ser = serial.Serial("/dev/ttyACM0", 57600, timeout=1)
        self.publisher_imu = self.create_publisher(Imu, 'imu_data', 10)
        self.publisher_sensor_imu = self.create_publisher(Float64MultiArray, 'sensor_data/gyro', 10)
        self.publisher_sensor_acc = self.create_publisher(Float64MultiArray, 'sensor_data/acc', 10)

    def read_and_publish(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            parts = line.split('\t')
            if len(parts) == 6:
                # -------- Implement Here
                
                # 
                self.get_logger().info(f'Publishing IMU Data: {line}')
            else:
                self.get_logger().error('Unexpected data format received.')


def main(args=None):
    rclpy.init(args=args)
    imu_serial_reader = IMUSerialReader()
    rclpy.spin(imu_serial_reader)
    imu_serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()