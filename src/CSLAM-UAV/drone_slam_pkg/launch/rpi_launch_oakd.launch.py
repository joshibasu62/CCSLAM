# !/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    
    use_sim_time = False 
    
    
    vslam_params = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        
        'subscribe_rgbd': True,
        'subscribe_depth': False, 
        'subscribe_imu': True,
        
        'approx_sync': True, 
        'queue_size': 20,
        'sync_queue_size': 20,
        
        'Odom/Strategy': '0',           
        'Vis/MinInliers': '15',         
        'Vis/MaxFeatures': '500',       
        'Rtabmap/DetectionRate': '1',   
        'Odom/ImageDecimation': '2',    
        
        'wait_imu_to_init': False,      
        'publish_tf': True,
        
    }

    return LaunchDescription([
        
        
        # ExecuteProcess(
        #     cmd=['MicroXRCEAgent', 'udp4', '--port', '8888'],
        #     output='screen'
        # ),

        Node(
            package='drone_slam_pkg', 
            executable='px4_imu_bridge', 
            name='px4_imu_converter',
            output='screen'
        ),

        
        # Node(
        #     package='depthai_ros_driver',
        #     executable='camera_node',
        #     name='oak_camera',
        #     output='screen',
        #     parameters=[os.path.expanduser('~/oak_run.yaml')],
        # ),
        #oak_rgb_camera_optical_frame

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'camera_rgb_camera_optical_frame']
        ),
        
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            namespace='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,

                'approx_sync': True, 
                'approx_sync_max_interval': 0.07,
                'queue_size': 20,
            }],
            remappings=[
                ('rgb/image', '/camera/rgb/image_raw'),
                ('rgb/camera_info', '/camera/rgb/camera_info'),
                ('depth/image', '/camera/stereo/image_raw'),
            ],
        ),

       
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            namespace='rtabmap',
            output='screen',
            parameters=[vslam_params],
            remappings=[
                ("imu", "/imu/data_converted"), 
                ('odom', '/odom')
            ],
        ),

       
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=[vslam_params],
            remappings=[
                ("imu", "/imu/data_converted"),
                ('odom', '/odom'), 
            ],
            arguments=['-d'], 
        ),
        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     name='rtabmap_viz',
        #     namespace='rtabmap',
        #     output='screen', 
        #     parameters=[vslam_params],
        #     remappings=[
        #         ("imu", "/imu/data_converted"),
        #         ('odom', '/odom'),
        #     ],
        # ),

        Node(
            package='drone_slam_pkg', 
            executable='visual_odom_converter', 
            name='visual_odom_converter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(
        #         get_package_share_directory('rtabmap_rviz_plugins'),
        #         'launch', 'rtabmap.rviz'
        #     )],
        #     parameters=[{'use_sim_time': True}]
        # ),
    ])