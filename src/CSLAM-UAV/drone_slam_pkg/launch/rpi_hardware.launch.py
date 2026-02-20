# !/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 1. Configuration
    use_sim_time = False 
    
    # RTAB-Map Parameters optimized for Raspberry Pi 4/5
    vslam_params = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        
        'subscribe_rgbd': True,
        'subscribe_depth': False, # We subscribe to RGBDWrapper, not raw depth here
        'subscribe_imu': True,
        
        'approx_sync': True, # True because IMU (FCU) and Camera are different devices
        'queue_size': 20,
        'sync_queue_size': 20,
        
        'Odom/Strategy': '0',           # 0=Frame-to-Frame (Fastest)
        'Vis/MinInliers': '15',         
        'Vis/MaxFeatures': '500',       # Reduced features for RPi CPU
        'Rtabmap/DetectionRate': '1',   # Update map at 1Hz to save CPU
        'Odom/ImageDecimation': '2',    # Process smaller images to speed up Odom
        
        'wait_imu_to_init': False,      # Don't hang waiting for IMU if topic delays
        'publish_tf': True,
    }

    return LaunchDescription([
        
        
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '--port', '8888'],
            output='screen'
        ),

        Node(
            package='drone_slam_pkg', # CHANGE THIS to your package name
            executable='px4_imu_bridge', # Ensure this is registered in setup.py or use full path
            name='px4_imu_converter',
            output='screen'
        ),

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
            ]),
            launch_arguments={
                'align_depth.enable': 'true',       # CRITICAL for RGB-D SLAM
                'pointcloud.enable': 'false',       # Save CPU, let RTABMap handle it
                'depth_module.profile': '640x480x15', # Lower res/fps for Pi
                'rgb_camera.profile': '640x480x15'
            }.items()
        ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),
        
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            namespace='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'approx_sync': False, 
                'queue_size': 20,
            }],
            remappings=[
                # Remap to the topics you listed in your logs
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
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
                ("imu", "/imu/data_converted"), # From our Python bridge
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
                ('odom', '/odom'), # Local odometry
            ],
            arguments=['-d'], # Delete old database
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace='rtabmap',
            output='screen',
            parameters=[vslam_params],
            remappings=[
                ("imu", "/imu/data_converted"),
                ('odom', '/odom'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('rtabmap_rviz_plugins'),
                'launch', 'rtabmap.rviz'
            )],
            parameters=[{'use_sim_time': True}]
        ),
    ])