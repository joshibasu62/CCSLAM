#!/usr/bin/env python3
############################################################################
# Simple PX4 Offboard position control example (ROS 2, px4_ros_com style)
#
# - Waits for preflight checks and /arm_message == True
# - Arms
# - Auto-takeoff to 2 m
# - Switches to OFFBOARD
# - Follows a 2x2 m square in local NED position
# - Commands land
############################################################################

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (QoSProfile,
                       QoSReliabilityPolicy,
                       QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from std_msgs.msg import Bool

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from rclpy.qos import qos_profile_system_default


class OffboardPositionControl(Node):

    def __init__(self):
        super().__init__('offboard_position_control')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',   # adjust if your topic name differs
            self.vehicle_status_callback,
            qos_profile)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)

        self.arm_msg_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile_system_default)

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)

        self.traj_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10)

        # Timers
        self.arm_timer_ = self.create_timer(0.1, self.arm_timer_callback)   # 10 Hz
        self.cmd_timer_ = self.create_timer(0.02, self.cmdloop_callback)   # 50 Hz

        # State variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.failsafe = False
        self.flightCheck = False

        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.myCnt = 0

        self.arm_message = False
        self.offboardMode = False

        # Local position
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.local_pos_valid = False

        # Mission / waypoints
        self.home_initialized = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0

        self.waypoints = []
        self.current_wp_idx = 0
        self.mission_finished = False

    # ----------------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------------
    def arm_message_callback(self, msg: Bool):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    def vehicle_status_callback(self, msg: VehicleStatus):
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        if msg.pre_flight_checks_pass != self.flightCheck:
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z
        self.local_pos_valid = msg.xy_valid and msg.z_valid

    # ----------------------------------------------------------------------
    # High-level state machine (arming, takeoff, mode, land)
    # ----------------------------------------------------------------------
    def arm_timer_callback(self):

        match self.current_state:
            case "IDLE":
                if self.flightCheck and self.arm_message:
                    self.current_state = "ARMING"
                    self.get_logger().info("Transition: IDLE -> ARMING")

            case "ARMING":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Arming failed: Flight check not passed")
                elif (self.arm_state == VehicleStatus.ARMING_STATE_ARMED
                      and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info("Transition: ARMING -> TAKEOFF")
                self.arm()

            case "TAKEOFF":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Takeoff aborted: Flight check failed")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    # When PX4 enters AUTO_TAKEOFF, we move to our LOITER state
                    self.current_state = "LOITER"
                    self.get_logger().info("Transition: TAKEOFF -> LOITER")
                self.arm()
                self.take_off()  # Takeoff to 2 m

            case "LOITER":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Loiter aborted: Flight check failed")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.current_state = "OFFBOARD"
                    self.get_logger().info("Transition: LOITER -> OFFBOARD")
                self.arm()

            case "OFFBOARD":
                if (not self.flightCheck or
                        self.arm_state != VehicleStatus.ARMING_STATE_ARMED or
                        self.failsafe):
                    self.current_state = "IDLE"
                    self.offboardMode = False
                    self.get_logger().info("Offboard aborted: safety condition")
                else:
                    # Send mode command to set PX4 to OFFBOARD
                    self.state_offboard()

                    # If mission is finished, go to LAND state
                    if self.mission_finished:
                        self.current_state = "LAND"
                        self.offboardMode = False
                        self.get_logger().info("Transition: OFFBOARD -> LAND")

            case "LAND":
                # Command landing until disarmed
                if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.land()
                else:
                    # Reset to IDLE when disarmed
                    self.current_state = "IDLE"
                    self.mission_finished = False
                    self.home_initialized = False
                    self.current_wp_idx = 0
                    self.get_logger().info("Landed and disarmed: back to IDLE")

        # Reset arm_message when disarmed so it doesn't auto-restart
        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm_message = False

        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(f"State: {self.current_state}")

        self.myCnt += 1

    # ----------------------------------------------------------------------
    # PX4 command helpers
    # ----------------------------------------------------------------------
    def state_offboard(self):
        self.myCnt = 0
        # main_mode=1, sub_mode=6 => OFFBOARD for PX4
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def take_off(self):
        # Takeoff to altitude 2 m (param7 is relative altitude in meters)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=1.0,
            param7=5.0)

    def land(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0,
                                param7=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # µs
        self.vehicle_command_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Offboard position control loop (waypoints)
    # ----------------------------------------------------------------------
    def cmdloop_callback(self):
        # Only run offboard setpoints while offboardMode is True
        if not self.offboardMode:
            return

        # Publish OffboardControlMode: position control
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.offboard_mode_pub.publish(offboard_msg)

        # Need a valid local position to run mission
        if not self.local_pos_valid:
            return

        # Initialize home and waypoints once after entering OFFBOARD
        if not self.home_initialized:
            self.home_x = self.local_x
            self.home_y = self.local_y
            self.home_z = self.local_z  # should be around -2.0 m after takeoff

            # Define 2 m square path at this altitude:
            # 1) straight 2 m (x+2),
            # 2) right 2 m (y+2),
            # 3) back 2 m (x-2),
            # 4) left 2 m (y-2) back to start
            self.waypoints = [
                (self.home_x,          self.home_y,          self.home_z),
                (self.home_x + 5.0,    self.home_y,          self.home_z),
                (self.home_x + 5.0,    self.home_y + 5.0,    self.home_z),
                (self.home_x,          self.home_y + 5.0,    self.home_z),
                (self.home_x,          self.home_y,          self.home_z),
            ]
            self.current_wp_idx = 0
            self.home_initialized = True
            self.get_logger().info("Waypoints initialized")

        # Current target waypoint
        target_x, target_y, target_z = self.waypoints[self.current_wp_idx]

        # Check distance to waypoint
        dx = self.local_x - target_x
        dy = self.local_y - target_y
        dz = self.local_z - target_z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        # If close enough, advance waypoint or finish mission
        waypoint_tolerance = 0.3  # meters
        if distance < waypoint_tolerance:
            if self.current_wp_idx < len(self.waypoints) - 1:
                self.current_wp_idx += 1
                self.get_logger().info(f"Reached waypoint {self.current_wp_idx}")
            else:
                # Last waypoint reached
                self.mission_finished = True

        # Publish TrajectorySetpoint for current target
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        # Position setpoint
        traj_msg.position[0] = float(target_x)
        traj_msg.position[1] = float(target_y)
        traj_msg.position[2] = float(target_z)

        # Don't use velocity/acceleration (set NaN)
        traj_msg.velocity[0] = float('nan')
        traj_msg.velocity[1] = float('nan')
        traj_msg.velocity[2] = float('nan')

        traj_msg.acceleration[0] = float('nan')
        traj_msg.acceleration[1] = float('nan')
        traj_msg.acceleration[2] = float('nan')

        # Keep yaw fixed (0 rad) and no yaw rate
        traj_msg.yaw = 0.0
        traj_msg.yawspeed = 0.0

        self.traj_setpoint_pub.publish(traj_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardPositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()