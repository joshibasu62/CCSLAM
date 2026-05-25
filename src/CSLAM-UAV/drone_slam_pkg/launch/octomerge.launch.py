import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Node(
        #     package='map_merge_3d',           
        #     executable='map_merge_3d_node',   
        #     name='map_merge_3d',
        #     output='screen',
        #     parameters=[{
        #         # Look for topics ending in "cloud_map" from your drones
        #         'robot_map_topic': 'cloud_map', 
                
        #         # The output topic for the combined 3D map
        #         'merged_map_topic': '/merged_3d_map', 
                
        #         # The global reference frame (matches your drone map frames)
        #         'world_frame': 'map',
                
        #         # Rates (in Hz)
        #         'compositing_rate': 0.3,
        #         'discovery_rate': 0.05,
        #         'estimation_rate': 0.01,
                
        #         'publish_tf': True
        #     }]
        # ),

        Node(
            package='octomap_server',          
            executable='octomap_server_node',     
            name='octomap_server',     
            output='screen',     
            parameters=[{         
                'use_sim_time': False,       # Changed for hardware!        
                'resolution': 0.05,         
                'frame_id': 'map',         
                
                'base_frame_id': 'world',         
                
                'pointcloud_min_z': 0.5,         
                'pointcloud_max_z': 1.0,         
                'occupancy_min_z': 0.5,         
                'occupancy_max_z': 1.0,         
                
                'latch': True,         
                'filter_ground': False,         
                'sensor_model.max_range': 5.0  
            }],     
            remappings=[         
                ('cloud_in', '/merged_3d_map'),  
                
                ('projected_map', '/merged_2d_map'),     
            ] 
        )
    ])