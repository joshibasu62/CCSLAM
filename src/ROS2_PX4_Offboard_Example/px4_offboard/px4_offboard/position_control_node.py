#!/usr/bin/env python3
############################################################################
# Position Control Node for PX4 Drone
# Sequence: Arm → Takeoff (2m) → Forward (2m) → Right (2m) → Right (2m) → Right (2m) → Land
############################################################################

__author__ = "Assistant"
__contact__ = ""

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import time

class PositionControl(Node):

    def __init__(self):
        super().__init__('position_control_node')
        
        # QoS Profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)
        
        self.arm_trigger_sub = self.create_subscription(
            Bool,
            '/arm_trigger',
            self.arm_trigger_callback,
            10)

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile)
        
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            "/fmu/in/vehicle_command", 
            10)

        # State machine variables
        self.current_state = "IDLE"
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.flight_check = False
        self.arm_trigger = False
        
        # Position variables (NED frame: North-East-Down)
        # Down is positive, so altitude = positive value
        self.current_position = [0.0, 0.0, 0.0]  # [x, y, z] in NED
        self.yaw = 0.0
        
        # Mission waypoints (relative to takeoff position)
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "z": -2.0, "yaw": 0.0, "desc": "Takeoff to 2m"},      # Takeoff
            {"x": 2.0, "y": 0.0, "z": -2.0, "yaw": 0.0, "desc": "Forward 2m"},         # Forward
            {"x": 2.0, "y": 2.0, "z": -2.0, "yaw": 0.0, "desc": "Right 2m"},          # Right 1
            {"x": 2.0, "y": 4.0, "z": -2.0, "yaw": 0.0, "desc": "Right 2m"},          # Right 2
            {"x": 2.0, "y": 6.0, "z": -2.0, "yaw": 0.0, "desc": "Right 2m"},          # Right 3
            {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "desc": "Land"}                # Land
        ]
        
        self.current_waypoint_index = 0
        self.waypoint_reached = False
        self.position_tolerance = 0.2  # meters
        self.waypoint_timeout = 30.0  # seconds
        self.waypoint_start_time = time.time()
        
        # Home position (set when arming)
        self.home_position = [0.0, 0.0, 0.0]
        self.home_set = False
        
        # Timers
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_callback)  # 10Hz
        self.control_timer = self.create_timer(0.05, self.control_callback)  # 20Hz

        self.get_logger().info("Position Control Node Initialized")

    def arm_trigger_callback(self, msg):
        """Callback for arm trigger"""
        self.arm_trigger = msg.data
        if self.arm_trigger:
            self.get_logger().info("Arm trigger received")

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status"""
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.flight_check = msg.pre_flight_checks_pass

    def local_position_callback(self, msg):
        """Callback for local position (NED frame)"""
        if msg.xy_valid and msg.z_valid:
            self.current_position = [msg.x, msg.y, msg.z]
            
            # Set home position when we get first valid position and not set yet
            if not self.home_set and self.current_state == "ARMING":
                self.home_position = [msg.x, msg.y, msg.z]
                self.home_set = True
                self.get_logger().info(f"Home position set: {self.home_position}")
            
            # Check if current waypoint is reached
            if self.current_state == "MISSION_RUNNING":
                self.check_waypoint_reached()

    def check_waypoint_reached(self):
        """Check if current waypoint is reached"""
        if self.current_waypoint_index >= len(self.waypoints):
            return
            
        target = self.waypoints[self.current_waypoint_index]
        dx = abs(self.current_position[0] - (self.home_position[0] + target["x"]))
        dy = abs(self.current_position[1] - (self.home_position[1] + target["y"]))
        dz = abs(self.current_position[2] - (self.home_position[2] + target["z"]))
        
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance < self.position_tolerance:
            if not self.waypoint_reached:
                self.waypoint_reached = True
                self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached: {target['desc']}")
        else:
            self.waypoint_reached = False

    def state_machine_callback(self):
        """Main state machine for the mission"""
        match self.current_state:
            case "IDLE":
                if self.arm_trigger and self.flight_check:
                    self.current_state = "ARMING"
                    self.get_logger().info("State: ARMING")
                    self.arm_vehicle()
            
            case "ARMING":
                if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.current_state = "TAKEOFF"
                    self.get_logger().info("State: TAKEOFF")
                    # Give some time before starting mission
                    time.sleep(2)
                    self.current_state = "MISSION_RUNNING"
                    self.current_waypoint_index = 0
                    self.waypoint_reached = False
                    self.waypoint_start_time = time.time()
                    self.get_logger().info("State: MISSION_RUNNING")
                    self.get_logger().info(f"Starting waypoint 0: {self.waypoints[0]['desc']}")
            
            case "MISSION_RUNNING":
                # Check timeout
                if time.time() - self.waypoint_start_time > self.waypoint_timeout:
                    self.get_logger().warn(f"Waypoint {self.current_waypoint_index} timeout")
                    self.current_state = "LANDING"
                
                # Check if current waypoint reached
                elif self.waypoint_reached:
                    self.current_waypoint_index += 1
                    
                    if self.current_waypoint_index >= len(self.waypoints):
                        self.current_state = "LANDING"
                        self.get_logger().info("State: LANDING - All waypoints completed")
                    else:
                        self.waypoint_reached = False
                        self.waypoint_start_time = time.time()
                        self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index}: {self.waypoints[self.current_waypoint_index]['desc']}")
            
            case "LANDING":
                self.land_vehicle()
                # Check if landed
                if abs(self.current_position[2]) < 0.3:  # Close to ground
                    self.current_state = "DISARMING"
                    self.get_logger().info("State: DISARMING")
            
            case "DISARMING":
                self.disarm_vehicle()
                if self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
                    self.current_state = "COMPLETED"
                    self.get_logger().info("State: COMPLETED - Mission finished successfully")

    def control_callback(self):
        """Control loop callback - publishes offboard control and setpoints"""
        if self.current_state in ["MISSION_RUNNING", "LANDING"]:
            # Publish offboard control mode
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = True  # Position control
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            self.publisher_offboard_mode.publish(offboard_msg)
            
            # Publish trajectory setpoint
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            
            if self.current_state == "MISSION_RUNNING" and self.current_waypoint_index < len(self.waypoints):
                target = self.waypoints[self.current_waypoint_index]
                trajectory_msg.position[0] = self.home_position[0] + target["x"]
                trajectory_msg.position[1] = self.home_position[1] + target["y"]
                trajectory_msg.position[2] = self.home_position[2] + target["z"]
                trajectory_msg.yaw = target["yaw"]
            
            elif self.current_state == "LANDING":
                # Land at home position
                trajectory_msg.position[0] = self.home_position[0]
                trajectory_msg.position[1] = self.home_position[1]
                trajectory_msg.position[2] = self.home_position[2]  # Ground level
                trajectory_msg.yaw = 0.0
            
            # Set velocity and acceleration to NaN (let position controller handle it)
            trajectory_msg.velocity = [float('nan'), float('nan'), float('nan')]
            trajectory_msg.acceleration = [float('nan'), float('nan'), float('nan')]
            trajectory_msg.yawspeed = float('nan')
            
            self.publisher_trajectory.publish(trajectory_msg)

    def arm_vehicle(self):
        """Send arm command"""
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_msg.param1 = 1.0  # Arm
        cmd_msg.target_system = 1
        cmd_msg.target_component = 1
        cmd_msg.source_system = 1
        cmd_msg.source_component = 1
        cmd_msg.from_external = True
        cmd_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(cmd_msg)
        self.get_logger().info("Arm command sent")

    def land_vehicle(self):
        """Send land command"""
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd_msg.param5 = self.home_position[0]  # Latitude (not used in local)
        cmd_msg.param6 = self.home_position[1]  # Longitude (not used in local)
        cmd_msg.param7 = self.home_position[2]  # Altitude
        cmd_msg.target_system = 1
        cmd_msg.target_component = 1
        cmd_msg.source_system = 1
        cmd_msg.source_component = 1
        cmd_msg.from_external = True
        cmd_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(cmd_msg)

    def disarm_vehicle(self):
        """Send disarm command"""
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_msg.param1 = 0.0  # Disarm
        cmd_msg.target_system = 1
        cmd_msg.target_component = 1
        cmd_msg.source_system = 1
        cmd_msg.source_component = 1
        cmd_msg.from_external = True
        cmd_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(cmd_msg)
        self.get_logger().info("Disarm command sent")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        position_control = PositionControl()
        rclpy.spin(position_control)
    except KeyboardInterrupt:
        position_control.get_logger().info("Shutting down position control node")
    finally:
        position_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()