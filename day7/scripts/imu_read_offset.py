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
        self.gyro_offset = [2.4800300000000113, -4.9763500000000045, -3.2608600000000294]
        self.cov_gyro = [0.004444143243243241, 0.0002054959959959966, -0.00021338758758758766, 
                         0.0002054959959959966, 0.0034772547547547482, 0.00016790690690690694, 
                         -0.00021338758758758766, 0.00016790690690690694, 0.0020553157157157096]
        self.cov_acc = [1.2891891891891953e-06, 9.888888888888985e-07, 9.349349349349354e-08, 
                         9.888888888888985e-07, 2.290190190190192e-06, 9.349349349349494e-08, 
                         9.349349349349354e-08, 9.349349349349494e-08, 7.971971971972041e-07]


    def read_and_publish(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            parts = line.split('\t')
            if len(parts) == 6:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"  # Adjust as needed
                # Gyroscope data in rad/s
                imu_msg.angular_velocity.x = float(parts[0]) - self.gyro_offset[0]
                imu_msg.angular_velocity.y = float(parts[1]) - self.gyro_offset[1]
                imu_msg.angular_velocity.z = float(parts[2]) - self.gyro_offset[2]
                imu_msg.angular_velocity_covariance = np.array( self.cov_gyro )
                # Accelerometer data in m/s^2
                imu_msg.linear_acceleration.x = float(parts[3]) * 9.81
                imu_msg.linear_acceleration.y = float(parts[4]) * 9.81
                imu_msg.linear_acceleration.z = float(parts[5]) * 9.81
                imu_msg.linear_acceleration_covariance = np.array( self.cov_acc ) * 9.81

                self.publisher_imu.publish(imu_msg)
                self.get_logger().info(f'Publishing IMU Data: {imu_msg}')
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