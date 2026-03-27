#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import SensorCombined  # <--- CHANGED THIS
from sensor_msgs.msg import Imu
import numpy as np

class PX4IMUBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')

        # QoS for PX4 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber to PX4 SensorCombined 
        self.subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.listener_callback,
            qos_profile)

        # Publisher to Standard ROS 2 IMU
        self.publisher = self.create_publisher(Imu, 'x500_drone_0/imu/data_converted', 10)

    def listener_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "x500_drone_0/imu_link" 
        
        # Acceleration 
        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = -float(msg.accelerometer_m_s2[1]) # Invert Y
        imu_msg.linear_acceleration.z = -float(msg.accelerometer_m_s2[2]) # Invert Z

        # Gyroscope 
        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = -float(msg.gyro_rad[1]) # Invert Y
        imu_msg.angular_velocity.z = -float(msg.gyro_rad[2]) # Invert Z
        
        # Orientation
        imu_msg.orientation.w = 1.0 # Identity quaternion
        
        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()