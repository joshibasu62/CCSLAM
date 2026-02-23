#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped

import tf2_ros
import tf_transformations
import numpy as np


class RosOdometry2VehicleOdometry(Node):

    def __init__(self):
        super().__init__('visual_odom_converter')

        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("repeat_odom", False)

        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.repeat_odom = self.get_parameter("repeat_odom").value

        # TF buffer
        cache_time = Duration(seconds=10.0)  # 10 seconds cache
        self.tf_buffer = tf2_ros.Buffer(cache_time=cache_time)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher (PX4 requires Best Effort QoS)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_profile
        )

        self.get_logger().info("ROS Odometry → PX4 VehicleOdometry node started")

    # ===============================
    # ENU → NED conversion helpers
    # ===============================

    def enu_to_ned_position(self, p):
        return np.array([p[0], -p[1], -p[2]])

    def enu_to_ned_quaternion(self, q):
        # q = [x, y, z, w] from ROS
        # Convert ENU → NED
        return np.array([q[0], -q[1], -q[2], q[3]])

    def enu_to_ned_vector(self, v):
        return np.array([v[0], -v[1], -v[2]])

    # ===============================
    # Callback
    # ===============================

    def odom_callback(self, msg: Odometry):

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.01)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        vehicle_msg = VehicleOdometry()

        # -----------------------------------
        # Timestamp (PX4 expects microseconds)
        # -----------------------------------
        now = self.get_clock().now()
        vehicle_msg.timestamp = int(now.nanoseconds / 1000)
        vehicle_msg.timestamp_sample = int(
            msg.header.stamp.sec * 1e6 +
            msg.header.stamp.nanosec / 1000
        )

        # -----------------------------------
        # Pose Frame
        # -----------------------------------
        vehicle_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        vehicle_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD

        # -----------------------------------
        # Position
        # -----------------------------------
        position_enu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        position_ned = self.enu_to_ned_position(position_enu)

        vehicle_msg.position = [
            float(position_ned[0]),
            float(position_ned[1]),
            float(position_ned[2])
        ]

        # -----------------------------------
        # Orientation
        # -----------------------------------
        q_ros = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        q_ned = self.enu_to_ned_quaternion(q_ros)

        vehicle_msg.q = [
            float(q_ned[3]),  # PX4 expects w first
            float(q_ned[0]),
            float(q_ned[1]),
            float(q_ned[2])
        ]

        # -----------------------------------
        # Linear Velocity (Body FRD)
        # -----------------------------------
        vel_enu = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        vel_frd = self.enu_to_ned_vector(vel_enu)

        vehicle_msg.velocity = [
            float(vel_frd[0]),
            float(vel_frd[1]),
            float(vel_frd[2])
        ]

        # -----------------------------------
        # Angular Velocity
        # -----------------------------------
        ang_enu = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        ang_frd = self.enu_to_ned_vector(ang_enu)

        vehicle_msg.angular_velocity = [
            float(ang_frd[0]),
            float(ang_frd[1]),
            float(ang_frd[2])
        ]

        # -----------------------------------
        # Covariances (optional but recommended)
        # -----------------------------------
        vehicle_msg.position_variance = [
            msg.pose.covariance[0],
            msg.pose.covariance[7],
            msg.pose.covariance[14]
        ]

        vehicle_msg.orientation_variance = [
            msg.pose.covariance[21],
            msg.pose.covariance[28],
            msg.pose.covariance[35]
        ]

        vehicle_msg.velocity_variance = [
            msg.twist.covariance[0],
            msg.twist.covariance[7],
            msg.twist.covariance[14]
        ]

        self.publisher.publish(vehicle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RosOdometry2VehicleOdometry()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
