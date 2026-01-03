#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined, VehicleOdometry
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PX4IMUBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.sub = self.create_subscription(SensorCombined,
                                            '/fmu/out/sensor_combined',
                                            self.callback, qos_profile )
        self.pub = self.create_publisher(Imu, '/imu/data', qos_profile)

        
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_cb,
            qos_profile
        )

        self.orientation = None

    def odom_cb(self, msg : VehicleOdometry):
        self.orientation = msg.q

    def callback(self, msg):
        imu_msg = Imu()
        # Timestamp (approximate)
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "x500_depth_0/camera_link/StereoOV7251" 


        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = float(msg.gyro_rad[1])
        imu_msg.angular_velocity.z = float(msg.gyro_rad[2])

        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.z = float(msg.accelerometer_m_s2[2])

        # No orientation data in SensorCombined → leave orientation unset (NaNs)
        # imu_msg.orientation_covariance[0] = -1  # marks "no orientation estimate"

        if self.orientation is not None:
            imu_msg.orientation.x = float(self.orientation[0])
            imu_msg.orientation.y = float(self.orientation[1])
            imu_msg.orientation.z = float(self.orientation[2])
            imu_msg.orientation.w = float(self.orientation[3])

            imu_msg.orientation_covariance[0] = 1e-6
        else:
            imu_msg.orientation_covariance[0] = -1

        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

