import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multirobot_map_merge',
            executable='map_merge',
            name='map_merge',
            output='screen',
            parameters=[{
                'robot_map_topic': 'map',
                'robot_namespace': 'x500_drone_',
                'merged_map_topic': 'global_map',
                'world_frame': 'world',
                'known_init_poses': False,
                'merging_rate': 2.0,
                'discovery_rate': 1.0,
                'estimation_rate': 0.5,
                
                # LOWERED CONFIDENCE to allow poor matches
                # Normal default is 1.0. Your logs showed ~0.2-0.3
                'estimation_confidence': 0.15, 
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_global',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])