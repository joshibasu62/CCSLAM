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

    # ══════════════════════════════════════════════════════════════════════
    #  OAK-D topic & frame reference
    #
    #  depthai_ros_driver publishes (camera_name = "camera" in oak_run.yaml):
    #
    #  Topics:
    #    /camera/rgb/image_raw              RGB image
    #    /camera/rgb/camera_info            RGB intrinsics
    #    /camera/stereo/image_raw           Stereo depth (aligned to RGB if configured)
    #    /camera/stereo/camera_info         Stereo depth intrinsics
    #
    #  TF tree (published by driver):
    #    camera_link
    #      └── camera_rgb_camera_frame
    #            └── camera_rgb_camera_optical_frame
    #      └── camera_left_camera_frame
    #            └── camera_left_camera_optical_frame
    #      └── camera_right_camera_frame
    #            └── camera_right_camera_optical_frame
    #
    #  We only add:  base_link → camera_link  (static, physical mounting)
    # ══════════════════════════════════════════════════════════════════════

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
        'queue_size': 200,
        'sync_queue_size': 100,

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
        # Core Infrastructure, Sensors, and TFs
        # ==========================================

        # ExecuteProcess(
        #     cmd=['micro-xrce-dds-agent', 'udp4', '--port', '8888'],
        #     output='screen'
        # ),

        Node(
            package='drone_slam_pkg',
            executable='px4_imu_bridge',
            name='px4_imu_converter',
            output='screen'
        ),

        # ── OAK-D Camera ──────────────────────────────────────────────────
        #    Replaces the RealSense IncludeLaunchDescription.
        #    Driver publishes internal TFs from camera_link downward.
        #    Make sure oak_run.yaml has:
        #      camera_name: "camera"
        #      stereo depth aligned to RGB (if available)
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak_camera',
            output='screen',
            parameters=[os.path.expanduser('~/oak_run.yaml')],
        ),

        # ── Static TF: base_link → camera_link ────────────────────────────
        #    OAK-D mounted 10 cm forward of base_link centre.
        #    The driver publishes: camera_link → camera_rgb_camera_frame
        #                                      → camera_rgb_camera_optical_frame
        #    Adjust x y z r p y to match your actual mounting.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['0.1', '0', '0', '0', '0', '0',
                        'base_link', 'camera_link'],
        ),

        # ── Static TF: base_link → imu_link ──────────────────────────────
        #    PX4 IMU co-located with base_link (adjust if offset).
        #    Needed because px4_imu_bridge publishes frame_id = "imu_link".
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['0', '0', '0', '0', '0', '0',
                        'base_link', 'imu_link'],
        ),

        # ── Static TF: base_link → middle ─────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_middle_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['0', '0', '0.14', '0', '0', '0',
                        'base_link', 'middle'],
        ),

        # ==========================================
        # PHASE 2: DELAYED START (10 Seconds)
        # IMU Processing
        # ==========================================
        TimerAction(
            period=10.0,
            actions=[
                # ── Madgwick IMU Filter ────────────────────────────────────
                #    /imu/data_converted (raw) → /imu/data (with orientation)
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

                # ── IMU → Stabilised TF ───────────────────────────────────
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
        # SLAM and Visual Odometry
        # ==========================================
        TimerAction(
            period=20.0,
            actions=[
                # ── RGBD Sync (OAK-D topics) ──────────────────────────────
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
                        ('rgb/image',       '/camera/rgb/image_raw'),
                        ('rgb/camera_info', '/camera/rgb/camera_info'),
                        ('depth/image',     '/camera/stereo/image_raw'),
                    ],
                ),

                # ── Visual Odometry ───────────────────────────────────────
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

                # ── SLAM ──────────────────────────────────────────────────
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

                # ── Point Cloud for Local Costmap (OAK-D topics) ──────────
                #    If stereo depth is aligned to RGB, use RGB camera_info.
                #    If NOT aligned, change to /camera/stereo/camera_info.
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
        # Nav2 and PX4 Odometry Relay
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

                # Node(
                #     package='px4_ros_com',
                #     executable='ros_odometry_to_vehicle_odometry_wo_map',
                #     name='odom_to_px4_drone_0',
                #     output='screen',
                #     parameters=[{
                #         'use_sim_time': use_sim_time,
                #         'repeat_odom': True,
                #         'odom_topic': '/rtabmap/odom',
                #         'vehicle_odometry_topic': '/fmu/in/vehicle_visual_odometry'
                #     }],
                # ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([nav2_launch]),
                    launch_arguments=[
                        ('use_sim_time', str(use_sim_time).lower()),
                        ('params_file', nav2_params_file),
                    ]
                ),
            ]
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/rviz/drone_0.rviz'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),
    ])