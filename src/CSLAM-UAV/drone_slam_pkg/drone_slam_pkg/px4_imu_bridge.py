#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleImu
from sensor_msgs.msg import Imu
import numpy as np

class PX4IMUBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')

        # QoS for PX4 (Must match the DDS Agent settings - usually Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber to PX4 data
        self.subscription = self.create_subscription(
            VehicleImu,
            '/fmu/out/vehicle_imu',
            self.listener_callback,
            qos_profile)

        # Publisher to Standard ROS 2 IMU
        self.publisher = self.create_publisher(Imu, '/imu/data_converted', 10)

    def listener_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link" # Treating FCU as base_link

        # PX4 (FRD) to ROS (ENU) conversion required usually, 
        # but for basic VSLAM, we map raw data. 
        # Acceleration (m/s^2)
        imu_msg.linear_acceleration.x = msg.accel[0]
        imu_msg.linear_acceleration.y = msg.accel[1]
        imu_msg.linear_acceleration.z = msg.accel[2]

        # Gyroscope (rad/s)
        imu_msg.angular_velocity.x = msg.gyro[0]
        imu_msg.angular_velocity.y = msg.gyro[1]
        imu_msg.angular_velocity.z = msg.gyro[2]
        
        # We don't have orientation (quaternion) from raw IMU, 
        # but RTAB-Map can use just Gyro+Accel for VIO.
        
        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()