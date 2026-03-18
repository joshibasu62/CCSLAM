#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time = False

    nav2_params_file = os.path.join(
        get_package_share_directory('drone_slam_pkg'), 'config',
        'nav2_params1.yaml'
    )

    pkg_nav2_bringup = get_package_share_directory('drone_slam_pkg')
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'custom_navigation_launch.py']
    )

    vslam_params = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'guess_frame_id': 'base_link_stabilized',

        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_odom': True,
        'subscribe_imu': True,
        'approx_sync': False,
        
        # FIXED: Changed queue_size to sync_queue_size
        'sync_queue_size': 100,
        'topic_queue_size': 10,

        'use_action_for_goal': True,

        'Odom/ResetCountdown': '1',
        'Vis/MinInliers': '15',
        'Odom/Strategy': '0',
        'wait_for_transform': 0.5,
        'Optimizer/GravitySigma': '0.1',
        'wait_imu_to_init': True,
        'publish_tf': True,

        'Grid/MinGroundHeight': '-0.1',
        'Grid/MapFrameProjection': 'true',
        'NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '1.15',
        'Grid/MaxObstacleHeight': '1.75',
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'Grid/RayTracing': 'true',
    }

    vslam_remappings = [
        ('imu', '/imu/data'),
        ('map', '/map'),
        ('navigate_to_pose', '/navigate_to_pose'),
        ('goal_pose', '/rtabmap_dummy_goal'),
        ('navigate_to_pose/_action/feedback',    '/navigate_to_pose/_action/feedback'),
        ('navigate_to_pose/_action/status',      '/navigate_to_pose/_action/status'),
        ('navigate_to_pose/_action/cancel_goal', '/navigate_to_pose/_action/cancel_goal'),
        ('navigate_to_pose/_action/get_result',  '/navigate_to_pose/_action/get_result'),
        ('navigate_to_pose/_action/send_goal',   '/navigate_to_pose/_action/send_goal'),
    ]

    return LaunchDescription([

        # ==========================================
        # PHASE 1: IMMEDIATE START (0 Seconds)
        # ==========================================

        Node(
            package='drone_slam_pkg',
            executable='px4_imu_bridge',
            name='px4_imu_converter',
            output='screen'
        ),

        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak_camera',
            output='screen',
            parameters=[os.path.expanduser('~/oak_run.yaml')],
        ),

        # FIXED: Updated arguments to modern ROS 2 syntax
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--x', '0.1', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_middle_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--x', '0', '--y', '0', '--z', '0.14', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'middle'],
        ),

        # ==========================================
        # PHASE 2: DELAYED START (10 Seconds)
        # ==========================================
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imu_filter',
                    output='screen',
                    parameters=[{
                        'use_mag': False,
                        'world_frame': 'enu',
                        'publish_tf': False,
                        'use_sim_time': use_sim_time,
                    }],
                    remappings=[
                        ('imu/data_raw', '/imu/data_converted'),
                    ],
                ),

                Node(
                    package='rtabmap_util',
                    executable='imu_to_tf',
                    name='imu_to_tf',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'fixed_frame_id': 'base_link_stabilized',
                        'base_frame_id': 'base_link',
                    }],
                    remappings=[
                        ('imu/data', '/imu/data'),
                    ],
                ),
            ]
        ),

        # ==========================================
        # PHASE 3: DELAYED START (20 Seconds)
        # ==========================================
        TimerAction(
            period=20.0,
            actions=[
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
                        # FIXED: Changed queue_size to sync_queue_size
                        'sync_queue_size': 20,
                    }],
                    remappings=[
                        ('rgb/image',       '/camera/rgb/image_raw'),
                        ('rgb/camera_info', '/camera/rgb/camera_info'),
                        ('depth/image',     '/camera/stereo/image_raw'),
                    ],
                ),

                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params, {'odom_frame_id': 'odom'}],
                    remappings=vslam_remappings,
                    arguments=['--ros-args', '--log-level', 'warn'],
                ),

                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=vslam_remappings,
                    arguments=['-d'],
                ),

                Node(
                    package='rtabmap_util',
                    executable='point_cloud_xyz',
                    name='point_cloud_xyz',
                    output='screen',
                    parameters=[{
                        'decimation': 2,
                        'max_depth': 3.0,
                        'voxel_size': 0.02,
                        'use_sim_time': use_sim_time,
                    }],
                    remappings=[
                        ('depth/image',       '/camera/stereo/image_raw'),
                        ('depth/camera_info', '/camera/rgb/camera_info'),
                        ('cloud',             '/camera/cloud'),
                    ],
                ),
            ]
        ),

        # ==========================================
        # PHASE 4: DELAYED START (45 Seconds)
        # ==========================================
        TimerAction(
            period=45.0,
            actions=[
                Node(
                    package='rtabmap_costmap_plugins',
                    executable='voxel_marker',
                    name='voxel_marker',
                    output='screen',
                    namespace='local_costmap',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([nav2_launch]),
                    launch_arguments=[
                        ('use_sim_time', str(use_sim_time).lower()),
                        ('params_file', nav2_params_file),
                    ]
                ),
            ]
        ),
    ])