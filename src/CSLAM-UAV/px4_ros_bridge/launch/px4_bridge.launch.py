from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_bridge',
            executable='px4_imu_bridge',
            name='px4_imu_bridge_node',
            output='screen'
        ),
        Node(
            package='px4_ros_bridge',
            executable='px4_odom_bridge',
            name='px4_odom_bridge_node',
            output='screen'
        )
    ])

