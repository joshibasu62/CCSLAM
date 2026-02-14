import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Transform, Quaternion
import math
import threading
import copy

from combine_grids.merging_pipeline import MergingPipeline

class MapSubscription:
    def __init__(self):
        self.map_sub = None
        self.map_updates_sub = None
        self.readonly_map = None # nav_msgs/OccupancyGrid
        self.initial_pose = Transform()
        self.lock = threading.Lock()

class MapMerge(Node):
    def __init__(self):
        super().__init__('map_merge')
        
        # Parameters
        self.declare_parameter('merging_rate', 4.0)
        self.declare_parameter('discovery_rate', 0.05)
        self.declare_parameter('estimation_rate', 0.5)
        self.declare_parameter('known_init_poses', True)
        self.declare_parameter('estimation_confidence', 1.0)
        self.declare_parameter('robot_map_topic', 'map')
        self.declare_parameter('robot_map_updates_topic', 'map_updates')
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('merged_map_topic', 'map')
        self.declare_parameter('world_frame', 'world')

        self.merging_rate = self.get_parameter('merging_rate').value
        self.discovery_rate = self.get_parameter('discovery_rate').value
        self.estimation_rate = self.get_parameter('estimation_rate').value
        self.known_init_poses = self.get_parameter('known_init_poses').value
        self.confidence_threshold = self.get_parameter('estimation_confidence').value
        self.robot_map_topic = self.get_parameter('robot_map_topic').value
        self.robot_map_updates_topic = self.get_parameter('robot_map_updates_topic').value
        self.robot_namespace_param = self.get_parameter('robot_namespace').value
        self.merged_map_topic = self.get_parameter('merged_map_topic').value
        self.world_frame = self.get_parameter('world_frame').value

        # State
        self.subscriptions = {} # Dict[robot_name, MapSubscription]
        self.pipeline = MergingPipeline()
        self.pipeline_lock = threading.Lock()

        # Publisher
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.merged_map_publisher = self.create_publisher(
            OccupancyGrid, self.merged_map_topic, qos_profile)

        # Timers
        self.create_timer(1.0 / self.merging_rate, self.map_merging)
        self.create_timer(1.0 / self.discovery_rate, self.topic_subscribing)
        
        if not self.known_init_poses:
            self.create_timer(1.0 / self.estimation_rate, self.pose_estimation)

        self.get_logger().info("MapMerge node initialized")

    def topic_subscribing(self):
        topic_names_and_types = self.get_topic_names_and_types()
        
        for name, types in topic_names_and_types:
            if 'nav_msgs/msg/OccupancyGrid' not in types:
                continue
                
            # Filter logic
            if not self.is_robot_map_topic(name):
                continue
            
            robot_name = self.robot_name_from_topic(name)
            if robot_name in self.subscriptions:
                continue
                
            # Init Pose Logic
            init_pose = Transform()
            init_pose.rotation.w = 1.0 # Identity default
            
            if self.known_init_poses:
                if not self.get_init_pose(robot_name, init_pose):
                    self.get_logger().warn(f"Could not get init pose for {robot_name}")
                    continue

            self.get_logger().info(f"Adding robot [{robot_name}] to system")
            
            sub_entry = MapSubscription()
            sub_entry.initial_pose = init_pose
            
            # Subscribe
            qos = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            
            # Capture robot_name in closure
            def map_cb(msg, rn=robot_name):
                self.full_map_update(msg, rn)
                
            sub_entry.map_sub = self.create_subscription(
                OccupancyGrid, name, map_cb, qos)
                
            # Updates subscription (optional)
            updates_topic = name.replace(self.robot_map_topic, self.robot_map_updates_topic)
            # Not fully implemented in this snippet to save space, but structure is here
            
            self.subscriptions[robot_name] = sub_entry

    def map_merging(self):
        grids = []
        transforms = []
        
        # Gather data
        for robot_name, sub in self.subscriptions.items():
            with sub.lock:
                if sub.readonly_map:
                    grids.append(sub.readonly_map)
                    transforms.append(sub.initial_pose)

        if not grids:
            return

        with self.pipeline_lock:
            self.pipeline.feed(grids)
            if self.known_init_poses:
                self.pipeline.set_transforms(transforms)
            
            merged_map = self.pipeline.compose_grids()

        if merged_map:
            merged_map.header.stamp = self.get_clock().now().to_msg()
            merged_map.header.frame_id = self.world_frame
            self.merged_map_publisher.publish(merged_map)

    def pose_estimation(self):
        grids = []
        for robot_name, sub in self.subscriptions.items():
            with sub.lock:
                if sub.readonly_map:
                    grids.append(sub.readonly_map)

        with self.pipeline_lock:
            self.pipeline.feed(grids)
            success = self.pipeline.estimate_transforms(confidence=self.confidence_threshold)
            if not success:
                self.get_logger().info("No grid poses estimated")

    def full_map_update(self, msg, robot_name):
        if robot_name not in self.subscriptions:
            return
        
        sub = self.subscriptions[robot_name]
        with sub.lock:
            sub.readonly_map = msg

    def is_robot_map_topic(self, topic):
        # Logic to check if topic matches robot_map_topic pattern
        # And contains robot_namespace if specified
        if self.merged_map_publisher.topic_name == topic:
            return False
            
        if self.robot_namespace_param and self.robot_namespace_param not in topic:
            return False
            
        return topic.endswith(self.robot_map_topic)

    def robot_name_from_topic(self, topic):
        # Assuming format /robot_name/map or robot_name/map
        parts = topic.split('/')
        # Crude extraction, assumes the part before 'map' is the name
        # A more robust regex would match ros1_names logic
        if topic.startswith('/'):
            return parts[1]
        return parts[0]

    def get_init_pose(self, robot_name, pose):
        # Look for parameters: robot_name.map_merge.init_pose_[x,y,z,yaw]
        prefix = f"{robot_name}.map_merge.init_pose_"
        
        try:
            # Note: In ROS2, getting undeclared parameters dynamically from other nodes 
            # is complex. Usually these should be declared on *this* node via a yaml file.
            # Here we try to fetch from this node's parameters assuming they were loaded via wildcard.
            
            # Since we allowed undeclared parameters in C++, we try to find them here.
            # In Python rclpy, we usually need to declare them or use list_parameters.
            # For simplicity, we assume they are passed as:
            # ros2 run map_merge map_merge --ros-args -p robot1.map_merge.init_pose_x:=1.0
            
            x_key = prefix + "x"
            y_key = prefix + "y"
            z_key = prefix + "z"
            yaw_key = prefix + "yaw"

            # Check if parameter exists in our node (loaded from yaml)
            if not self.has_parameter(x_key):
                self.declare_parameter(x_key, 0.0)
                self.declare_parameter(y_key, 0.0)
                self.declare_parameter(z_key, 0.0)
                self.declare_parameter(yaw_key, 0.0)

            pose.translation.x = self.get_parameter(x_key).value
            pose.translation.y = self.get_parameter(y_key).value
            pose.translation.z = self.get_parameter(z_key).value
            yaw = self.get_parameter(yaw_key).value

            # Euler to Quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            pose.rotation.w = cy
            pose.rotation.z = sy
            pose.rotation.x = 0.0
            pose.rotation.y = 0.0
            
            return True
        except Exception as e:
            self.get_logger().warn(f"Param error: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = MapMerge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()