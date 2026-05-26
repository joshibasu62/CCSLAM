from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_slam_pkg',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_base_frame': 'base_link',
                'map_frame':        'map',
                'costmap_topic':         'global_costmap/costmap',
                'costmap_updates_topic': 'global_costmap/costmap_updates',
                'planner_frequency':  0.33,  
                'progress_timeout':   30.0,  
                'min_frontier_size':  0.75,  
                'blacklist_radius':   0.5,    
                'gain_scale':      1.0,
                'potential_scale': 3.0,
                'visualize': True,
            }],
        ),
    ])