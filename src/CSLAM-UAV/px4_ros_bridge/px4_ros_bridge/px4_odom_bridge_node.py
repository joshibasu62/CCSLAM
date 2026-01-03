# px4_to_odom_bridge.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from builtin_interfaces.msg import Time


class PX4ToOdomBridge(Node):
    def __init__(self):
        super().__init__('px4_to_odom_bridge')

        # Create a custom QoS profile to match PX4's publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match PX4's default
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.get_logger().info('PX4 VehicleOdometry -> /odom bridge started')

    def odom_callback(self, msg: VehicleOdometry):
        odom_msg = Odometry()

        # Header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position and orientation
        odom_msg.pose.pose.position.x = float(msg.position[0])
        odom_msg.pose.pose.position.y = float(msg.position[1])
        odom_msg.pose.pose.position.z = float(msg.position[2])
        odom_msg.pose.pose.orientation.x = float(msg.q[0])
        odom_msg.pose.pose.orientation.y = float(msg.q[1])
        odom_msg.pose.pose.orientation.z = float(msg.q[2])
        odom_msg.pose.pose.orientation.w = float(msg.q[3])

        # Pose covariance (row-major)
        odom_msg.pose.covariance = [
            float(msg.position_variance[0]), 0, 0, 0, 0, 0,
            0, float(msg.position_variance[1]), 0, 0, 0, 0,
            0, 0, float(msg.position_variance[2]), 0, 0, 0,
            0, 0, 0, float(msg.orientation_variance[0]), 0, 0,
            0, 0, 0, 0, float(msg.orientation_variance[1]), 0,
            0, 0, 0, 0, 0, float(msg.orientation_variance[2])
        ]

        # Linear and angular velocity
        odom_msg.twist.twist.linear.x = float(msg.velocity[0])
        odom_msg.twist.twist.linear.y = float(msg.velocity[1])
        odom_msg.twist.twist.linear.z = float(msg.velocity[2])

        odom_msg.twist.twist.angular.x = float(msg.angular_velocity[0])
        odom_msg.twist.twist.angular.y = float(msg.angular_velocity[1])
        odom_msg.twist.twist.angular.z = float(msg.angular_velocity[2])

        # Velocity covariance (same as above)
        odom_msg.twist.covariance = [
            float(msg.velocity_variance[0]), 0, 0, 0, 0, 0,
            0, float(msg.velocity_variance[1]), 0, 0, 0, 0,
            0, 0, float(msg.velocity_variance[2]), 0, 0, 0,
            0, 0, 0, 1e-6, 0, 0,
            0, 0, 0, 0, 1e-6, 0,
            0, 0, 0, 0, 0, 1e-6
        ]

        self.publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4ToOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


