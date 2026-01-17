#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import time

class SimpleCloudMerger(Node):
    def __init__(self):
        super().__init__('cloud_merger')

        # Parameters
        self.declare_parameter('drone_0_topic', '/x500_drone_0/cloud_map')
        self.declare_parameter('drone_1_topic', '/x500_drone_1/cloud_map')
        self.declare_parameter('output_topic', '/combined_map')
        self.declare_parameter('target_frame', 'world')

        # Internal storage for the latest clouds
        self.cloud_0 = None
        self.cloud_1 = None

        # TF Buffer to handle coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(PointCloud2, 
            self.get_parameter('drone_0_topic').value, self.cb_cloud_0, 10)
        self.create_subscription(PointCloud2, 
            self.get_parameter('drone_1_topic').value, self.cb_cloud_1, 10)

        # Publisher
        self.pub_combined = self.create_publisher(PointCloud2, 
            self.get_parameter('output_topic').value, 10)

        # Timer (1 Hz merge rate)
        self.create_timer(1.0, self.merge_clouds)
        self.get_logger().info("Cloud Merger Started. Waiting for maps...")

    def cb_cloud_0(self, msg):
        self.cloud_0 = msg

    def cb_cloud_1(self, msg):
        self.cloud_1 = msg

    def merge_clouds(self):
        if self.cloud_0 is None and self.cloud_1 is None:
            return

        clouds_to_merge = []
        target_frame = self.get_parameter('target_frame').value

        # Process Cloud 0
        if self.cloud_0:
            try:
                # Get transform from drone's map frame to World
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    self.cloud_0.header.frame_id,
                    rclpy.time.Time())
                # Transform the cloud
                transformed_c0 = do_transform_cloud(self.cloud_0, transform)
                clouds_to_merge.append(transformed_c0)
            except Exception as e:
                self.get_logger().warn(f"Could not transform Cloud 0: {e}")

        # Process Cloud 1
        if self.cloud_1:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    self.cloud_1.header.frame_id,
                    rclpy.time.Time())
                transformed_c1 = do_transform_cloud(self.cloud_1, transform)
                clouds_to_merge.append(transformed_c1)
            except Exception as e:
                self.get_logger().warn(f"Could not transform Cloud 1: {e}")

        if not clouds_to_merge:
            return

        # Combine points
        # Note: Ideally, we use an efficient library, but for visualization, 
        # generating a list of points from both and creating a new PC2 is "okay" for small maps.
        # For large RTAB-Maps, simply republishing them in the same frame 
        # allows RViz to show them together without physically merging the arrays.
        
        # However, to physically merge into one message:
        all_points = []
        for cloud in clouds_to_merge:
            # Read points (x, y, z, rgb)
            gen = pc2.read_points(cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            all_points.extend(list(gen))

        if not all_points:
            return

        # Create Header
        header = clouds_to_merge[0].header
        header.frame_id = target_frame
        header.stamp = self.get_clock().now().to_msg()

        # Create combined PointCloud2
        combined_cloud = pc2.create_cloud(header, 
            [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            all_points
        )
        
        self.pub_combined.publish(combined_cloud)
        # self.get_logger().info(f"Published merged map with {len(all_points)} points")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()