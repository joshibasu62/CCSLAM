#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
import math

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
        # We need Odometry to get current Yaw
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

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
        self.current_yaw = 0.0
        self.target_height = -2.0  # Flight altitude (NED, so negative is up)
        
        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.offboard_set = False

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def odom_callback(self, msg):
        # Extract Yaw from Quaternion (PX4 q is [w, x, y, z])
        q = msg.q
        # yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

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

        # 2. Convert Nav2 Body Frame (Forward/Left) to PX4 NED Frame (North/East)
        # Nav2 X (Forward) -> Cos(Yaw)*X - Sin(Yaw)*Y
        # Nav2 Y (Left)    -> Sin(Yaw)*X + Cos(Yaw)*Y
        
        # ROS Input
        vx_ros = self.current_vel.linear.x
        vy_ros = self.current_vel.linear.y
        wz_ros = self.current_vel.angular.z

        # Coordinate Rotation
        # PX4 NED X (North)
        vel_north = vx_ros * math.cos(self.current_yaw) - vy_ros * math.sin(self.current_yaw)
        # PX4 NED Y (East)
        vel_east  = vx_ros * math.sin(self.current_yaw) + vy_ros * math.cos(self.current_yaw)

        traj_msg = TrajectorySetpoint()
        traj_msg.position = [float('nan'), float('nan'), float('nan')]
        
        # Velocity in NED
        traj_msg.velocity = [
            vel_north, 
            vel_east, 
            0.0 # Velocity Control for XY, Position for Z
        ]
        
        # Yaw Rate (Inverted because NED Z is down, ROS Z is up)
        traj_msg.yawspeed = -wz_ros 

        # Altitude Hold
        traj_msg.position[2] = self.target_height 

        self.traj_setpoint_pub.publish(traj_msg)

        # 3. Auto-Arm and Takeoff Logic
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