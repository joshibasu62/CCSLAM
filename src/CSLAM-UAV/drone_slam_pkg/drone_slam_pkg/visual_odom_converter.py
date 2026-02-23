#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

import numpy as np

class VisualOdomConverter(Node):
    def __init__(self):
        super().__init__('visual_odom_converter')

        # QoS for PX4
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: Listen to RTAB-Map / Visual SLAM odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Ensure this matches your launch file topic
            self.odom_callback,
            10
        )

        # Publisher: Send to PX4
        self.vehicle_odom_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_profile
        )

    def odom_callback(self, msg):
        vehicle_odom = VehicleOdometry()

        # 1. Timestamp sync
        vehicle_odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vehicle_odom.timestamp_sample = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3)

        # 2. Frame definition (PX4 uses NED)
        # POSE_FRAME_NED = 1
        # VELOCITY_FRAME_NED = 1
        vehicle_odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        vehicle_odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # 3. Coordinate Conversion: FLU (ROS) -> NED (PX4)
        # Position: x=x, y=-y, z=-z
        vehicle_odom.position = [
            msg.pose.pose.position.x,
            -msg.pose.pose.position.y,
            -msg.pose.pose.position.z
        ]

        # Orientation: Quaternion (w, x, y, z) -> (w, x, -y, -z)
        vehicle_odom.q = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            -msg.pose.pose.orientation.y,
            -msg.pose.pose.orientation.z
        ]

        # Velocity: x=x, y=-y, z=-z
        vehicle_odom.velocity = [
            msg.twist.twist.linear.x,
            -msg.twist.twist.linear.y,
            -msg.twist.twist.linear.z
        ]

        # Angular Velocity: x=x, y=-y, z=-z
        vehicle_odom.angular_velocity = [
            msg.twist.twist.angular.x,
            -msg.twist.twist.angular.y,
            -msg.twist.twist.angular.z
        ]

        # Variances (Optional but recommended for EKF fusion)
        # If SLAM is confident, keep these low. If lost, these usually jump up.
        vehicle_odom.position_variance = [0.1, 0.1, 0.1]
        vehicle_odom.velocity_variance = [0.1, 0.1, 0.1]
        vehicle_odom.orientation_variance = [0.05, 0.05, 0.05]

        self.vehicle_odom_pub.publish(vehicle_odom)

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdomConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()