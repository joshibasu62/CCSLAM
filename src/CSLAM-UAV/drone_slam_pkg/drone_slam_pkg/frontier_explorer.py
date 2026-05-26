#!/usr/bin/env python3
"""
Frontier-based exploration node for ROS2 Jazzy + Nav2.

Architecture:
  - Subscribes to Nav2's global costmap (OccupancyGrid + incremental updates)
  - Detects frontier cells: free (0) cells adjacent to unknown (-1) cells
  - Clusters frontier cells into regions using connected-component labeling
  - Scores each region: gain_scale * size  -  potential_scale * distance
  - Sends the best frontier as a NavigateToPose action goal to Nav2
  - Monitors progress; blacklists frontiers on timeout or abort
  - Publishes MarkerArray for RViz visualisation when 'visualize' is True
"""

import math
from typing import List, Optional, Tuple

import numpy as np
from scipy import ndimage

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Frontier:
    """A cluster of frontier cells represented by its centroid and cell count."""

    __slots__ = ('centroid', 'size', 'score')

    def __init__(self, centroid: Tuple[float, float], size: int):
        self.centroid: Tuple[float, float] = centroid  # (x, y) metres, map frame
        self.size: int = size                           # number of frontier cells
        self.score: float = 0.0



class FrontierExplorer(Node):

    # Costmap cell semantics (OccupancyGrid int8)
    _FREE     = 0
    _OCCUPIED = 65    # treat 65 as occupied (Nav2 uses 65 as lethal)
    _UNKNOWN  = -1

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('robot_base_frame',        'base_link')
        self.declare_parameter('map_frame',               'map')
        self.declare_parameter('costmap_topic',           'global_costmap/costmap')
        self.declare_parameter('costmap_updates_topic',   'global_costmap/costmap_updates')
        self.declare_parameter('visualize',               True)
        self.declare_parameter('planner_frequency',       0.33)   # Hz
        self.declare_parameter('progress_timeout',        30.0)   # seconds
        self.declare_parameter('potential_scale',         3.0)    # distance penalty weight
        self.declare_parameter('gain_scale',              1.0)    # frontier size reward weight
        self.declare_parameter('orientation_scale',       0.0)    # unused, kept for API compat
        self.declare_parameter('transform_tolerance',     0.3)    # TF lookup timeout 
        self.declare_parameter('min_frontier_size',       0.75)   # metres (converted to cells)
        self.declare_parameter('blacklist_radius',        0.5)    # metres around failed goals

        p = self.get_parameters([
            'robot_base_frame', 
            'map_frame',
            'costmap_topic', 
            'costmap_updates_topic',
            'visualize', 
            'planner_frequency', 
            'progress_timeout',
            'potential_scale', 
            'gain_scale', 
            'transform_tolerance',
            'min_frontier_size', 
            'blacklist_radius',
        ])

        (self._robot_frame, self._map_frame,
         self._costmap_topic, self._costmap_updates_topic,
         self._visualize, self._planner_freq, self._progress_timeout,
         self._potential_scale, self._gain_scale, self._tf_tolerance,
         self._min_frontier_size, self._blacklist_radius) = [x.value for x in p]

        # State
        self._costmap: Optional[OccupancyGrid] = None
        self._map_data: Optional[np.ndarray]   = None   # shape (H, W), dtype int8

        self._current_goal: Optional[Tuple[float, float]] = None
        self._goal_handle   = None
        self._last_progress = self.get_clock().now()
        self._exploring     = False

        self._blacklist: List[Tuple[float, float]] = []

        # TF
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Subscribers
        self.create_subscription(
            OccupancyGrid, self._costmap_topic,
            self._costmap_cb, 10)
        self.create_subscription(
            OccupancyGridUpdate, self._costmap_updates_topic,
            self._costmap_update_cb, 10)

        # Action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Visualization
        if self._visualize:
            self._marker_pub = self.create_publisher(MarkerArray, 'frontier_markers', 10)

        # Main loop
        self.create_timer(1.0 / self._planner_freq, self._explore_cb)

        self.get_logger().info(
            f'FrontierExplorer ready  |  costmap: {self._costmap_topic}'
            f'  |  frequency: {self._planner_freq:.2f} Hz')

    # Costmap callbacks

    def _costmap_cb(self, msg: OccupancyGrid):
        self._costmap  = msg
        self._map_data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.get_logger().info(
            f'Costmap received: {msg.info.width}x{msg.info.height} cells '
            f'@ {msg.info.resolution:.3f} m/cell', once=True)

    def _costmap_update_cb(self, msg: OccupancyGridUpdate):
        """Apply an incremental patch from the costmap update topic."""
        if self._costmap is None or self._map_data is None:
            return
        r0, c0 = msg.y, msg.x
        h,  w  = msg.height, msg.width
        patch = np.array(msg.data, dtype=np.int8).reshape(h, w)
        self._map_data[r0:r0 + h, c0:c0 + w] = patch

    # Robot pose

    def _robot_pos(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_tolerance))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None

    # Frontier detection

    def _detect_frontiers(self) -> List[Frontier]:
        """
        Returns a list of Frontier objects found in the current costmap.

        Algorithm:
          1. Build boolean masks for free and unknown cells.
          2. Dilate the unknown mask by 1 cell (4 connected).
          3. Frontier mask = free AND (dilated unknown).
          4. Label connected components one Frontier per component.
          5. Drop components smaller than min_frontier_size.
        """
        data = self._map_data
        info = self._costmap.info
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        free_mask    = (data == self._FREE)
        unknown_mask = (data == self._UNKNOWN)

        # 4 connected dilation of unknown into free space
        struct           = ndimage.generate_binary_structure(2, 1)
        unknown_dilated  = ndimage.binary_dilation(unknown_mask, structure=struct)
        frontier_mask    = free_mask & unknown_dilated

        labeled, n_labels = ndimage.label(frontier_mask, structure=struct)

        min_cells = max(1, int(math.ceil(self._min_frontier_size / res)))
        frontiers: List[Frontier] = []

        for lbl in range(1, n_labels + 1):
            cells = np.argwhere(labeled == lbl)   # shape (N, 2): [row, col]
            if len(cells) < min_cells:
                continue

            centroid_row = float(cells[:, 0].mean())
            centroid_col = float(cells[:, 1].mean())
            cx = ox + centroid_col * res
            cy = oy + centroid_row * res

            frontiers.append(Frontier(centroid=(cx, cy), size=len(cells)))

        return frontiers

    # Scoring & blacklist

    def _score(self, f: Frontier, robot: Tuple[float, float]) -> float:
        """
        score = gain_scale * (size * resolution)   ← information gain proxy
              - potential_scale * euclidean_distance

        Maximise → prefer large nearby frontiers.
        """
        dx   = f.centroid[0] - robot[0]
        dy   = f.centroid[1] - robot[1]
        dist = max(math.hypot(dx, dy), 0.01)

        gain = self._gain_scale * f.size * self._costmap.info.resolution
        cost = self._potential_scale * dist
        return gain - cost

    def _blacklisted(self, f: Frontier) -> bool:
        for bl in self._blacklist:
            if math.hypot(f.centroid[0] - bl[0], f.centroid[1] - bl[1]) \
                    < self._blacklist_radius:
                return True
        return False

    # Navigation

    def _send_goal(self, x: float, y: float):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server not available yet.')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'→ Frontier goal: ({x:.2f}, {y:.2f})')

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

        self._current_goal  = (x, y)
        self._last_progress = self.get_clock().now()
        self._exploring     = True

    def _goal_response_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self._reset_state()
            return
        self._goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        # Any feedback means Nav2 is still making progress
        self._last_progress = self.get_clock().now()

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ Frontier reached.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('✗ Goal aborted — blacklisting frontier.')
            if self._current_goal:
                self._blacklist.append(self._current_goal)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal cancelled.')
        self._reset_state()

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._reset_state()

    def _reset_state(self):
        self._current_goal = None
        self._goal_handle  = None
        self._exploring    = False

    # Main exploration loop

    def _explore_cb(self):
        if self._costmap is None or self._map_data is None:
            return

        robot = self._robot_pos()
        if robot is None:
            return

        # Check progress timeout
        if self._exploring and self._current_goal is not None:
            elapsed = (self.get_clock().now() - self._last_progress).nanoseconds / 1e9
            if elapsed > self._progress_timeout:
                self.get_logger().warn(
                    f'Progress timeout after {elapsed:.1f}s — blacklisting & replanning.')
                self._blacklist.append(self._current_goal)
                self._cancel_goal()
            else:
                return      # still navigating, nothing to do

        # Detect frontiers
        frontiers = self._detect_frontiers()

        if not frontiers:
            self.get_logger().info(
                'No frontiers found — exploration complete (or map not ready).',
                throttle_duration_sec=10.0)
            return

        # Filter blacklisted
        valid = [f for f in frontiers if not self._blacklisted(f)]
        if not valid:
            self.get_logger().warn(
                'All frontiers are blacklisted — clearing blacklist and retrying.')
            self._blacklist.clear()
            return

        # Score
        for f in valid:
            f.score = self._score(f, robot)
        best = max(valid, key=lambda f: f.score)

        if self._visualize:
            self._publish_markers(valid, best)

        # Navigate
        self._send_goal(best.centroid[0], best.centroid[1])

    # Visualization

    def _publish_markers(self, frontiers: List[Frontier], best: Frontier):
        ma   = MarkerArray()
        now  = self.get_clock().now().to_msg()

        # Delete all previous markers in one shot
        clear        = Marker()
        clear.header.frame_id = self._map_frame
        clear.header.stamp    = now
        clear.ns     = 'frontiers'
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, f in enumerate(frontiers):
            m                   = Marker()
            m.header.frame_id   = self._map_frame
            m.header.stamp      = now
            m.ns                = 'frontiers'
            m.id                = i + 1
            m.type              = Marker.SPHERE
            m.action            = Marker.ADD
            m.pose.position.x   = f.centroid[0]
            m.pose.position.y   = f.centroid[1]
            m.pose.position.z   = 0.5          # lift above ground for drone viz
            m.pose.orientation.w = 1.0

            is_best = (f is best)
            size    = 0.5 if is_best else 0.3
            m.scale.x = m.scale.y = m.scale.z = size
            m.color   = (ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)   # green  → selected
                         if is_best else
                         ColorRGBA(r=0.1, g=0.5, b=1.0, a=0.6))  # blue   → candidate
            ma.markers.append(m)

            # Score label
            txt                  = Marker()
            txt.header.frame_id  = self._map_frame
            txt.header.stamp     = now
            txt.ns               = 'frontier_scores'
            txt.id               = i + 1
            txt.type             = Marker.TEXT_VIEW_FACING
            txt.action           = Marker.ADD
            txt.pose.position.x  = f.centroid[0]
            txt.pose.position.y  = f.centroid[1]
            txt.pose.position.z  = 0.8
            txt.pose.orientation.w = 1.0
            txt.scale.z          = 0.25
            txt.color            = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            txt.text             = f'{f.score:.1f} ({f.size})'
            ma.markers.append(txt)

        self._marker_pub.publish(ma)



def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()