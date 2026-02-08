#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class PX4VelBridge(Node):
    def __init__(self):
        super().__init__('px4_vel_bridge')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos_profile)

        # Publishers
        self.offboard_ctrl_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_vel = Twist()
        self.target_height = -2.0  # Flight altitude (NED, so negative is up)
        
        # Timer for heartbeat (must be > 2Hz)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.offboard_set = False

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmd_vel_callback(self, msg):
        self.current_vel = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Switching to Offboard mode")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def timer_callback(self):
        # 1. Publish Offboard Control Mode Heartbeat
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_ctrl_mode_pub.publish(offboard_msg)

        # 2. Convert cmd_vel (ENU) to TrajectorySetpoint (NED)
        # ROS ENU: X=Forward, Y=Left, Z=Up
        # PX4 NED: X=North(Forward), Y=East(Right), Z=Down
        traj_msg = TrajectorySetpoint()
        traj_msg.position = [float('nan'), float('nan'), float('nan')]
        
        # Map ROS Linear X to PX4 X (Forward)
        # Map ROS Linear Y to PX4 Y (Right = -Left)
        # Map ROS Linear Z to PX4 Z (Down = -Up)
        
        # NOTE: Nav2 gives velocity in Map/Base frame. 
        # For simplicity in simple offboard, we send velocity in body frame or local frame.
        # Here we assume cmd_vel is relative to the drone's heading (standard Nav2 output)
        
        traj_msg.velocity = [
            self.current_vel.linear.x, 
            -self.current_vel.linear.y, 
            0.0 # Don't let nav2 control vertical velocity directly, hold height
        ]
        
        # Yaw Rate
        traj_msg.yawspeed = -self.current_vel.angular.z # ROS CCW + / PX4 CW + check this based on SITL behavior

        # Altitude Hold Logic (Simple P-Controller to stay at target_height)
        # We enforce position control for Z, velocity for XY
        traj_msg.position[2] = self.target_height 

        self.traj_setpoint_pub.publish(traj_msg)

        # 3. Auto-Arm and Takeoff Logic (simplistic)
        # If we are receiving commands and not in offboard, try to switch
        if self.offboard_set == False and self.current_vel.linear.x != 0.0:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_set = True

def main(args=None):
    rclpy.init(args=args)
    node = PX4VelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()