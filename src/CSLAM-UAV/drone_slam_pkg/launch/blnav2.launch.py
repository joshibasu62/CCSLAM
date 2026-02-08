#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
    # Update path to your URDF
    urdf_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'
    # Update path to where you saved the yaml file from Step 2
    nav2_params_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/config/nav2_params.yaml'
    # Update path to where you saved the python bridge from Step 1
    # bridge_script_path = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/scripts/px4_vel_bridge.py'

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    vslam_params = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        
        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_imu': True,
        'approx_sync': False, 
        'queue_size': 200,
        'sync_queue_size': 100,
        
        'Odom/ResetCountdown': '1',     
        'Vis/MinInliers': '15',         
        'Odom/Strategy': '0',           
        'wait_for_transform': 0.2,
        'Optimizer/GravitySigma': '0.3',
        'wait_imu_to_init': True,
        'publish_tf': True,

        # CRITICAL FOR NAV2: Generate 2D map from 3D camera data
        'Grid/3D': 'true', 
        'Grid/RayTracing': 'true',
        'Grid/FromDepth': 'true', # Create occupancy grid from depth camera
        'Grid/MaxGroundHeight': '0.2', # Anything below 20cm is ground
        'Grid/MaxObstacleHeight': '2.0', # Ignore ceiling
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '2',
    }

    return LaunchDescription([
        # 1. PX4 SITL
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth'],
            output='screen',
            shell=True
        ),
        # 2. QGroundControl
        ExecuteProcess(
            cmd=['gnome-terminal', '--', ' ./QGroundControl-x86_64.AppImage'],
            cwd=os.path.expanduser('~/Downloads'),
            output='screen',
            shell=True
        ),
        # 3. MicroXRCEAgent
        ExecuteProcess(
             cmd=['MicroXRCEAgent','udp4', '--port', '8888']
        ),

        TimerAction(
            period=2.0,
            actions=[
                # 4. ROS-Gz Bridge
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
                
                # 5. Robot State Publisher
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': open(urdf_file).read()},
                                {'use_sim_time': True}]
                ),
                
                # 6. RTAB-Map Sync
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

                # 7. RTAB-Map Odom
                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                    ],
                ),

                # 8. RTAB-Map SLAM (Provides /map and map->odom TF)
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ('odom', '/odom'),
                        ('map', '/map'),
                    ],
                    # arguments=['-d'], # Uncomment to delete DB on start
                ),

                Node(
                    package='rtabmap_viz',
                    executable='rtabmap_viz',
                    name='rtabmap_viz',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ('odom', '/odom'),
                    ],
                ),

                # 9. CUSTOM BRIDGE: Cmd_vel -> PX4 Trajectory
                Node(
                    package='drone_slam_pkg', # REPLACE with your actual package name if different
                    executable='px4_vel_bridge', # Ensure this file is executable and in CMakeLists if using C++ or installed python script
                    name='px4_vel_bridge',
                    output='screen',
                    # If you just have the script file and not installed as a node:
                    # executable=bridge_script_path 
                ),

                # 10. Nav2 Bringup
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

                # 11. RViz (Nav2 Config)
                # Note: Switched to Nav2 default view, or you can use your custom one
                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        )
    ])