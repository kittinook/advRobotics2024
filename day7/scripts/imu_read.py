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
        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0


        # [INFO] [1707379980.026784490] [imu_calibration_node]: Calibration complete: 
        # GX offset: 2.0350999999999995, 
        # GY offset: 0.051550000000000006, 
        # GZ offset: -1.6954200000000001

    def read_and_publish(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            parts = line.split('\t')
            if len(parts) == 6:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"  # Adjust as needed
                # Gyroscope data in rad/s
                imu_msg.angular_velocity.x = float(parts[0])
                imu_msg.angular_velocity.y = float(parts[1])
                imu_msg.angular_velocity.z = float(parts[2])
                # imu_msg.angular_velocity_covariance = np.array( [   0.001, 0.001, 0.001, 
                #                                                     0.001, 0.001, 0.001, 
                #                                                     0.001, 0.001, 0.001,])
                # Accelerometer data in m/s^2
                imu_msg.linear_acceleration.x = float(parts[3])
                imu_msg.linear_acceleration.y = float(parts[4])
                imu_msg.linear_acceleration.z = float(parts[5])
                # imu_msg.linear_acceleration_covariance = np.array( [   0.001, 0.001, 0.001, 
                #                                                     0.001, 0.001, 0.001, 
                #                                                     0.001, 0.001, 0.001,])

                sensor_gyro_msg = Float64MultiArray()
                sensor_gyro_msg.data.append(float(parts[0]))
                sensor_gyro_msg.data.append(float(parts[1]))
                sensor_gyro_msg.data.append(float(parts[2]))
                sensor_acc_msg = Float64MultiArray()
                sensor_acc_msg.data.append(float(parts[3]))
                sensor_acc_msg.data.append(float(parts[4]))
                sensor_acc_msg.data.append(float(parts[5]))
                    
                self.publisher_imu.publish(imu_msg)
                self.publisher_sensor_imu.publish(sensor_gyro_msg)
                self.publisher_sensor_acc.publish(sensor_acc_msg)
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