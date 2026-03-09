# !/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
    urdf_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'

    vslam_params = {
        'use_sim_time': True,
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
        
        'Odom/ResetCountdown': '1',     
        'Vis/MinInliers': '15',         
        'Odom/Strategy': '0',           
        'wait_for_transform': 0.2,
        'Optimizer/GravitySigma': '0.1',
        'wait_imu_to_init': True,
        'publish_tf': True,
        # 'Vis/FeatureType': '10',
        # 'Kp/DetectorStrategy': '10',

        'Grid/MinGroundHeight': '-0.1',
        'Grid/MapFrameProjection': 'true',
        'NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '1.15', 
        'Grid/MaxObstacleHeight': '1.75',
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '5',
    }

    return LaunchDescription([
        ExecuteProcess(
             cmd=['MicroXRCEAgent','udp4', '--port', '8888']
        ),

        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth'],
            output='screen',
            shell=True
        ),

        ExecuteProcess(
            cmd=['gnome-terminal', '--', ' ./QGroundControl-x86_64.AppImage'],
            cwd=os.path.expanduser('~/Downloads'),
            output='screen',
            shell=True
        ),
        
        TimerAction(
            period=1.0,
            actions=[
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
                    parameters=[{'robot_description': open(urdf_file).read()},
                                {'use_sim_time': True}]
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
                        # Output: /imu/data (filtered with orientation)
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

                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ("imu", "/imu/data"),  
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
                        ("imu", "/imu/data"),  
                        ('odom', 'rtabmap/odom'),
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
                        ("imu", "/imu/data"),  
                        ('odom', 'rtabmap/odom'),
                    ],
                ),
                
                # ===== SEND ODOMETRY TO PX4 =====
                # Node(
                #     package='px4_ros_com', 
                #     executable='ros_odometry_to_vehicle_odometry', 
                #     name='ros_odometry_to_vehicle_odometry',
                #     output='screen',
                #     parameters=[{
                #         'use_sim_time': True,
                #         'repeat_odom': True,
                #     }],
                #     remappings=[
                #         ('odom', '/rtabmap/odom'), 
                #     ],
                # ),

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

                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/rviz/drone_0.rviz'],
                    parameters=[{'use_sim_time': True}]
                ),

                Node(
                    package='px4_offboard',
                    namespace='px4_offboard',
                    executable='control',
                    name='control',
                    prefix='gnome-terminal --',
                ),

                Node(
                    package='px4_offboard',
                    namespace='px4_offboard',
                    executable='velocity_control',
                    name='velocity'
                ),
            ]
        )])