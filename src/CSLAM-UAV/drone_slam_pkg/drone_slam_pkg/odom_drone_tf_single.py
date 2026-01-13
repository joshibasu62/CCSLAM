#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleOdometry
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_system_default

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_drone_tf')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile_system_default
        )

         # Publisher for path
        self.path_pub = self.create_publisher(Path, 'drone_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

    def odom_callback(self, msg: VehicleOdometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Position
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])

        # Orientation
        quat = [float(q) for q in msg.q]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


        # Append current pose to path
        pose = PoseStamped()
        pose.header.stamp = t.header.stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation
        self.path_msg.poses.append(pose)

        # Publish path
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

