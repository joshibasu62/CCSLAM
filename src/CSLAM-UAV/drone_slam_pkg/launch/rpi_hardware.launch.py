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
        'subscribe_odom': True,
        'subscribe_imu': True,
        'approx_sync': False, # Sync done in rgbd_sync
        'queue_size': 200,
        'sync_queue_size': 100,
        
        'Odom/ResetCountdown': '1',     
        'Vis/MinInliers': '15',         
        'Odom/Strategy': '0',           
        'wait_for_transform': 0.2,
        'Optimizer/GravitySigma': '0.3',
        'wait_imu_to_init': True,
        'publish_tf': True,

        # 'Grid/3D': True,
        # 'Grid/RayTracing': True,
        'Grid/MinGroundHeight': '-0.1',
        'Grid/MapFrameProjection': 'true',
        'NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.1', 
        'Grid/MaxObstacleHeight': '1.75',
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '5',
        
        # 'database_path': f'~/.ros/{db_name}.db'
    }

    return LaunchDescription([
        
        
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '--port', '8888'],
            output='screen'
        ),

        Node(
            package='drone_slam_pkg', 
            executable='px4_imu_bridge', 
            name='px4_imu_converter',
            output='screen'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
            ]),
            launch_arguments={
                'align_depth.enable': 'true',      
                'pointcloud.enable': 'false',       
                'depth_module.profile': '640x480x15', 
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
                ("imu", "/imu/data_converted"), 
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
            package='px4_ros_com', 
            executable='ros_odometry_to_vehicle_odometry_wo_map', 
            name='ros_odometry_to_vehicle_odometry_wo_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'repeat_odom': True,
                            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/rviz/drone_0.rviz'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])