#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
    urdf_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'

    
    nav2_params_file = os.path.join(
        get_package_share_directory('drone_slam_pkg'), 'config',
        'nav2_params.yaml'
    )

    # joy_config_file_path = os.path.join(
    #     get_package_share_directory('drone_slam_pkg'), 'config',
    #     'joy_config.yaml'
    # )
    
    pkg_nav2_bringup = get_package_share_directory('drone_slam_pkg') 
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'custom_navigation_launch.py']
    )

    vslam_params = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'guess_frame_id': 'base_link_stabilized',

        'subscribe_rgbd': True,
        'subscribe_depth': False,
        # 'subscribe_odom_info': True,   #  from subscribe_odom
        'subscribe_odom': True,        #  from subscribe_odom
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
        # 'RGBD/StartAtOrigin': 'true', 
    }

    vslam_remappings = [
        ('imu', '/imu/data'),
        ('map', '/map'),                          
        ('navigate_to_pose', '/navigate_to_pose'), 
        ('goal_pose', '/rtabmap_dummy_goal'),
        # For Humble compatibility (https://github.com/ros2/ros2/issues/1312):
        ('navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/feedback'),
        ('navigate_to_pose/_action/status', '/navigate_to_pose/_action/status'),
        ('navigate_to_pose/_action/cancel_goal', '/navigate_to_pose/_action/cancel_goal'),
        ('navigate_to_pose/_action/get_result', '/navigate_to_pose/_action/get_result'),
        ('navigate_to_pose/_action/send_goal', '/navigate_to_pose/_action/send_goal'),
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '--port', '8888']
        ),

        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth'],
            output='screen',
            shell=True
        ),

        ExecuteProcess(
            cmd=['gnome-terminal', '--', './QGroundControl-x86_64.AppImage'],
            cwd=os.path.expanduser('~/Downloads'),
            output='screen',
            shell=True
        ),

        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_rgbd_imu',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                "/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            ],
            remappings=[
                ("/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu", "/x500_drone_0/imu/data"),
            ],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_file).read()},
                {'use_sim_time': True}
            ]
        ),

        # Node(package='teleop_twist_joy', executable='teleop_node', output='screen',
        #     parameters=[joy_config_file_path,
        #                 {'use_sim_time': True}],
        # ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '0', '0', '0.14',
                '0', '0', '0',
                'base_link', 'middle'
            ],
        ),

        
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
                'use_sim_time': True,
            }],
            remappings=[
                ('imu/data_raw', '/x500_drone_0/imu/data'),
            ],
        ),

        Node(
            package='rtabmap_util',
            executable='imu_to_tf',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'fixed_frame_id': 'base_link_stabilized',
                'base_frame_id': 'base_link',
            }],
            remappings=[
                ('imu/data', '/imu/data'),
            ],
        ),

        
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            namespace='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.04,
                'queue_size': 200,
                'sync_queue_size': 100,
            }],
            remappings=[
                ('rgb/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/depth_camera'),
            ],
        ),

        # Visual Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            namespace='rtabmap',
            output='screen',
            parameters=[vslam_params, {'odom_frame_id': 'odom'}],
            remappings=vslam_remappings,  
            arguments=["--ros-args", "--log-level", 'warn'],
        ),

        # SLAM
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

        # Visualization
        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     name='rtabmap_viz',
        #     namespace='rtabmap',
        #     output='screen',
        #     parameters=[vslam_params],
        #     remappings=vslam_remappings,   # 
        # ),

        
        # Generates filtered point cloud for local costmap
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            output='screen',
            parameters=[{
                'decimation': 2,
                'max_depth': 3.0,
                'voxel_size': 0.02,
                'use_sim_time': True,
            }],
            remappings=[
                ('depth/image', '/depth_camera'),
                ('depth/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                ('cloud', '/camera/cloud'),
            ],
        ),

        
        Node(
            package='rtabmap_costmap_plugins',
            executable='voxel_marker',
            output='screen',
            namespace='local_costmap',
            parameters=[{'use_sim_time': True}],
        ),

        
        Node(
            package='px4_ros_com',
            executable='ros_odometry_to_vehicle_odometry_wo_map',
            name='ros_odometry_to_vehicle_odometry_wo_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'repeat_odom': True,
            }],
            remappings=[
                ('odom', '/rtabmap/odom'),
            ],
        ),

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('params_file', nav2_params_file),
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/rviz/simulation.rviz'],
            parameters=[{'use_sim_time': True}]
        ),

        
    

        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package = 'px4_ros_com',
                    executable = 'offboard_control',
                    name = 'offboard_control',
                    output = 'screen',
                    parameters = [{
                        'use_sim_time': True,
                    }],
                )
            ]
        )
    ])