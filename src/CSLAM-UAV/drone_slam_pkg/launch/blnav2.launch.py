#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
    
    # Paths
    urdf_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'
    nav2_params_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/config/nav2_params.yaml'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # SLAM Parameters
    vslam_params = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_imu': True,
        'approx_sync': False, 
        'wait_imu_to_init': True,
        'publish_tf': True, # RTAB-Map publishes Map->Odom
        
        # Optimization
        'Odom/Strategy': '0',           
        'Odom/ResetCountdown': '1',     
        'Vis/MinInliers': '15',
        'Optimizer/GravitySigma': '0.1', # Adjusted based on reference
        
        # Grid Generation (For 2D Map)
        'Grid/RayTracing': 'true',
        'Grid/FromDepth': 'true', 
        'Grid/MaxGroundHeight': '0.2',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '2',
    }

    return LaunchDescription([
        # 1. PX4 SITL & QGC
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth'],
            output='screen', shell=True
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', ' ./QGroundControl-x86_64.AppImage'],
            cwd=os.path.expanduser('~/Downloads'),
            output='screen', shell=True
        ),
        ExecuteProcess(cmd=['MicroXRCEAgent','udp4', '--port', '8888']),

        TimerAction(
            period=2.0,
            actions=[
                # 2. ROS-Gz Bridge
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_bridge_rgbd_imu',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                    arguments=[
                        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        # Camera
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                        # IMU
                        "/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    ],
                    remappings=[
                        ("/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu", "/x500_drone_0/imu/data"),
                    ],
                ),
                
                # 3. Robot State Publisher
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': open(urdf_file).read()},
                                {'use_sim_time': True}]
                ),
                
                # 4. Point Cloud Generator (CRITICAL FOR NAV2 COSTMAP)
                # This converts the depth image to a pointcloud that Nav2 can use for 3D avoidance
                Node(
                    package='rtabmap_util', 
                    executable='point_cloud_xyz', 
                    output='screen',
                    parameters=[{
                        'decimation': 2,
                        'max_depth': 4.0,
                        'voxel_size': 0.05,
                        'use_sim_time': True
                    }],
                    remappings=[
                        ('depth/image', '/depth_camera'),
                        ('depth/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                        ('cloud', '/camera/cloud') # Output topic for Nav2
                    ]
                ),

                # 5. RTAB-Map Sync
                Node(
                    package='rtabmap_sync',
                    executable='rgbd_sync',
                    name='rgbd_sync',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'approx_sync': True, 
                        'queue_size': 20,
                    }],
                    remappings=[
                        ('rgb/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                        ('rgb/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                        ('depth/image', '/depth_camera'),
                    ],
                ),

                # 6. RTAB-Map Odometry
                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        # Publishes /rtabmap/odom by default
                    ],
                ),

                # 7. RTAB-Map SLAM
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ('odom', '/rtabmap/odom'), # Subscribe to visual odometry
                        ('map', '/map'),
                    ],
                ),

                # 8. Custom Bridge (Modified for Frame Rotation)
                Node(
                    package='drone_slam_pkg',
                    executable='px4_vel_bridge', 
                    name='px4_vel_bridge',
                    output='screen',
                ),

                # 9. Nav2 Bringup
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'True',
                        'params_file': nav2_params_file,
                        'autostart': 'True',
                    }.items()
                ),

                # 10. RViz
                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    # You might want to save a config that visualizes /camera/cloud and /map
                    arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        )
    ])