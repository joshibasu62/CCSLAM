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
    drone_ns = 'x500_drone_1'

    # nav2_params_file = os.path.join(
    #     get_package_share_directory('drone_slam_pkg'), 'config',
    #     f'{drone_ns}_nav2_params1.yaml'
    # )

    pkg_nav2_bringup = get_package_share_directory('drone_slam_pkg')
    
    # nav2_launch = PathJoinSubstitution(
    #     [pkg_nav2_bringup, 'launch', f'{drone_ns}_custom_navigation_launch.py']
    # )

    def get_vslam_params(drone_namespace, db_name='rtabmap.db'):
        return {
            'use_sim_time': use_sim_time,
            'frame_id': f'{drone_namespace}/base_link',
            'map_frame_id': f'{drone_namespace}/map',
            'odom_frame_id': f'{drone_namespace}/odom',
            'guess_frame_id': f'{drone_namespace}/base_link_stabilized',

            'subscribe_rgbd': True,
            'subscribe_depth': False,
            'subscribe_odom': True,
            'subscribe_imu': True,
            'approx_sync': False,
            'queue_size': 10,
            'sync_queue_size': 10,

            'use_action_for_goal': True,

            'Odom/ResetCountdown': '1',
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1', 
            'Odom/Strategy': '0',
            # 'Odom/ImageDecimation': '2',
            'Vis/MaxFeatures': '300',
            # 'Vis/EstimationType': '1',
            'Kp/DetectorStrategy': '10',

            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',

            'wait_for_transform': 2.0,
            'Optimizer/GravitySigma': '0.1',
            'wait_imu_to_init': False,
            'publish_tf': True,

            'Grid/MinGroundHeight': '-0.1',
            # 'Grid/MapFrameProjection': 'true',
            'NormalsSegmentation': 'true',
            'Grid/MaxGroundHeight': '0.5',
            'Grid/MaxObstacleHeight': '1.0',
            'Grid/NoiseFilteringRadius': '0.15',
            'Grid/NoiseFilteringMinNeighbors': '7',
            'Grid/RayTracing': 'true',
        }

    # Generate the params dynamically using the function
    vslam_params = get_vslam_params(drone_ns)

    vslam_remappings = [
        ('imu', f'/{drone_ns}/imu/data'),
        ('map', f'/{drone_ns}/map'),
        ('navigate_to_pose', f'/{drone_ns}/navigate_to_pose'),
        ('goal_pose', f'/{drone_ns}/rtabmap_dummy_goal'),
        ('navigate_to_pose/_action/feedback',    f'/{drone_ns}/navigate_to_pose/_action/feedback'),
        ('navigate_to_pose/_action/status',      f'/{drone_ns}/navigate_to_pose/_action/status'),
        ('navigate_to_pose/_action/cancel_goal', f'/{drone_ns}/navigate_to_pose/_action/cancel_goal'),
        ('navigate_to_pose/_action/get_result',  f'/{drone_ns}/navigate_to_pose/_action/get_result'),
        ('navigate_to_pose/_action/send_goal',   f'/{drone_ns}/navigate_to_pose/_action/send_goal'),
    ]

    return LaunchDescription([

        Node(
            package='drone_slam_pkg',
            executable='px4_imu_bridge1',
            name='px4_imu_converter1',
            output='screen'
        ),

        # Node(
        #     package='depthai_ros_driver',
        #     executable='camera_node',
        #     name='oak_camera',
        #     namespace=drone_ns,
        #     output='screen',
        #     parameters=[os.path.expanduser('~/oak_run1.yaml')],
        #     remappings=[
        #         (IMAGE_TOPIC,  f'/{drone_ns}/color/image_raw'),
        #         (INFO_TOPIC,   f'/{drone_ns}/color/camera_info'),
        #         (STEREO_TOPIC, f'/{drone_ns}/stereo/image_raw'),
        #     ]
        # ),

        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_color_image',
            namespace=drone_ns,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('image',       f'/{drone_ns}/color/image_raw'),
                ('camera_info', f'/{drone_ns}/color/camera_info'),
                ('image_rect',  f'/{drone_ns}/rgb/image_rect') 
            ],
        ),
          
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            namespace=drone_ns,
            arguments=['0.12', '0.03', '0.242', '0', '0', '0',
                        f'{drone_ns}/base_link', f'{drone_ns}/camera_link'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            namespace=drone_ns,
            arguments=['0', '0', '0', '0', '0', '0',
                        f'{drone_ns}/base_link', f'{drone_ns}/imu_link'],
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_camera_optical_tf',
        #     namespace=drone_ns,
        #     arguments=['0.1', '0', '0', '0', '0', '0', 
        #                 f'{drone_ns}/base_link', f'{drone_ns}/camera_rgb_camera_optical_frame']
        # ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_middle_tf',
            namespace=drone_ns,
            arguments=['0', '0', '0', '0', '0', '0',
                        f'{drone_ns}/base_link', f'{drone_ns}/middle'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical_tf',
            namespace=drone_ns,
            arguments=[
                '0', '0', '0',
                '-1.5708', '0', '-1.5708',
                f'{drone_ns}/camera_link', 'camera_rgb_camera_optical_frame'
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imu_filter',
                    namespace=drone_ns,
                    output='screen',
                    parameters=[{
                        'use_mag': False,
                        'world_frame': 'enu',
                        'publish_tf': False,
                        'use_sim_time': use_sim_time,
                    }],
                    remappings=[
                        ('imu/data_raw', f'/{drone_ns}/imu/data_converted'),
                        ('imu/data', f'/{drone_ns}/imu/data'),
                    ],
                ),

                Node(
                    package='rtabmap_util',
                    executable='imu_to_tf',
                    name='imu_to_tf',
                    namespace=drone_ns,
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'fixed_frame_id': f'{drone_ns}/base_link_stabilized',
                        'base_frame_id': f'{drone_ns}/base_link',
                        'wait_for_transform': 1.0,  
                    }],
                    remappings=[
                        ('imu/data', f'/{drone_ns}/imu/data'),
                    ],
                ),
            ]
        ),

        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='rtabmap_sync',
                    executable='rgbd_sync',
                    name='rgbd_sync',
                    namespace=f'{drone_ns}/rtabmap',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'approx_sync': False,
                        # 'approx_sync_max_interval': 0.04,
                        'queue_size': 10,
                        'sync_queue_size': 10,
                        
                    }],
                    remappings=[
                        ('rgb/image',       f'/{drone_ns}/rgb/image_rect'),
                        ('rgb/camera_info', f'/{drone_ns}/color/camera_info'),
                        ('depth/image',     f'/{drone_ns}/stereo/image_raw'),
                    ],
                ),

                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    namespace=f'{drone_ns}/rtabmap',
                    output='screen',
                    parameters=[vslam_params, {'odom_frame_id': f'{drone_ns}/odom'}],
                    remappings=vslam_remappings,
                    arguments=['--ros-args', '--log-level', 'info'],
                ),

                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    namespace=f'{drone_ns}/rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=vslam_remappings,
                    arguments=['-d'],
                ),

                Node(
                    package='rtabmap_util',
                    executable='point_cloud_xyz',
                    name='point_cloud_xyz',
                    namespace=drone_ns,
                    output='screen',
                    parameters=[{
                        'decimation': 2,
                        'max_depth': 3.0,
                        'voxel_size': 0.02,
                        'use_sim_time': use_sim_time,
                        
                    }],
                    remappings=[
                        ('depth/image',       f'/{drone_ns}/stereo/image_raw'),
                        ('depth/camera_info', f'/{drone_ns}/color/camera_info'),
                        ('cloud',             f'/{drone_ns}/camera/cloud'),
                    ],
                ),
            ]
        ),

        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='px4_ros_com',
                    executable='ros_odometry_to_vehicle_odometry',
                    name='odom_to_px4_drone_0',
                    namespace=drone_ns,
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'repeat_odom': False,
                        'map_frame_id': f'/{drone_ns}/map',
                        'odom_topic': f'/{drone_ns}/rtabmap/odom',
                        'vehicle_odometry_topic': '/px4_1/fmu/in/vehicle_visual_odometry'
                    }],
                ),
            ],
        ),

        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package='rtabmap_costmap_plugins',
                    executable='voxel_marker',
                    name='voxel_marker',
                    output='screen',
                    namespace=f'{drone_ns}/local_costmap',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),

                # IncludeLaunchDescription(
                #     PythonLaunchDescriptionSource([nav2_launch]),
                #     launch_arguments=[
                #         ('namespace', drone_ns),
                #         ('use_sim_time', str(use_sim_time).lower()),
                #         ('params_file', nav2_params_file),
                #     ]
                # ),
            ]
        ),
    ])