#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = False

    # Exact paths from your original script
    nav2_params_file = os.path.join(
        get_package_share_directory('drone_slam_pkg'), 'config',
        'nav2_params1.yaml'
    )

    pkg_nav2_bringup = get_package_share_directory('drone_slam_pkg')
    
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'custom_navigation_launch.py']
    )

    return LaunchDescription([
        
        # Voxel marker for the local costmap (from your TimerAction block)
        Node(
            package='rtabmap_costmap_plugins',
            executable='voxel_marker',
            name='voxel_marker',
            output='screen',
            namespace='local_costmap',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # The exact Nav2 inclusion block you had commented out
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments=[
                ('use_sim_time', str(use_sim_time).lower()),
                ('params_file', nav2_params_file),
            ]
        ),
    ])