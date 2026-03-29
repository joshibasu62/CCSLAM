import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',
            executable='ros_odometry_to_vehicle_odometry_wo_map',
            name='odom_to_px4_drone_0',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'repeat_odom': True,
                'odom_topic': '/x500_drone_0/odom',
                # 'map_frame_id': 'x500_drone_0/map',  # Ensure this matches your RTAB-Map output
                'vehicle_odometry_topic': '/fmu/in/vehicle_visual_odometry'
            }],
        ),
        Node(
            package='px4_ros_com',
            executable='ros_odometry_to_vehicle_odometry_wo_map',
            name='odom_to_px4_drone_1',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'repeat_odom': True,
                'odom_topic': '/x500_drone_1/odom',
                # 'map_frame_id': 'x500_drone_1/map',  # Ensure this matches your RTAB-Map output
                'vehicle_odometry_topic': 'px4_1/fmu/in/vehicle_visual_odometry'
            }],
        ),
    ])