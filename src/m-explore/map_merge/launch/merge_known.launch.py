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
                'known_init_poses': True,
                'merging_rate': 2.0,
                'discovery_rate': 1.0,
                'estimation_rate': 0.5,
                'estimation_confidence': 1.0,

                # Drone 0 (Origin)
                # Note: Because of your C++ fix, these parameters will be found correctly
                'x500_drone_0/map_merge/init_pose_x': 0.0,
                'x500_drone_0/map_merge/init_pose_y': 0.0,
                'x500_drone_0/map_merge/init_pose_z': 0.0,
                'x500_drone_0/map_merge/init_pose_yaw': 0.0,

                # Drone 1 (Offset by -0.8m in Y)
                'x500_drone_1/map_merge/init_pose_x': 0.0,
                'x500_drone_1/map_merge/init_pose_y': -0.8,
                'x500_drone_1/map_merge/init_pose_z': 0.0,
                'x500_drone_1/map_merge/init_pose_yaw': 0.0,
            }]
        ),
        
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_global',
            output='screen',
            parameters=[{'use_sim_time': True}],
            # You can add arguments=['-d', 'path/to/config.rviz'] if you have one
        )
    ])