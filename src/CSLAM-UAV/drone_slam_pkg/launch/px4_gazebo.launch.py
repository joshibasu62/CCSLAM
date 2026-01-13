



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

    common_parameters = {
        'subscribe_depth': True,
        'subscribe_rgbd': False,
        'subscribe_stereo': False,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'subscribe_imu': True,
        'use_sim_time': True,
        'always_check_imu_tf': False,
        'load_db': False,
        'wait_imu_to_init': True,
        'approx_sync_max_interval': 0.04,
        'queue_size': 200,
        'sync_queue_size': 100,
        'topic_queue_size': 20  # Increased topic buffer

    }
    return LaunchDescription([
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
        ExecuteProcess(
             cmd=['MicroXRCEAgent','udp4', '--port', '8888']
        ),
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_bridge_rgbd_imu',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                    arguments=[
                        # Simulation clock
                        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

                        # RGB camera
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",

                        # Camera info
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",

                        # Depth camera raw image
                        "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",

                        # Depth camera as PointCloud2
                        "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",

                        # IMU
                        "/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    ]
                ),
                # Node(
                #      package='px4_ros_bridge',
                #      executable='px4_imu_bridge',
                #      output='screen',
                #      parameters=[{'use_sim_time': True}]
                #      ),
                # Node(
                #      package='px4_ros_bridge',
                #      executable='px4_odom_bridge',
                #      output='screen',
                #      parameters=[{'use_sim_time': True}]
                #      ),
                # Node(
                #     package='drone_slam_pkg',
                #     executable='odom_drone_tf',
                #     output='screen'
                #     ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': open(urdf_file).read()},
                                {'use_sim_time': True}
                                ]
                ),
                # Node(
                #         package='rtabmap_odom',
                #         executable='rgbd_odometry',
                #         output='screen',
                #         parameters=[{
                #             **common_parameters, 
                #             'frame_id': 'base_link',
                #             'odom_frame_id': 'odom',
                #             'publish_tf': True
                #         }],
                #         remappings=[
                #             ('rgb/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                #             ('rgb/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                #             ('depth/image', '/depth_camera'),
                #             ('imu', '/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
                #         ]
                #     ),
                Node(
                    package='drone_slam_pkg',
                    executable='odom_drone_tf',
                    name='odom_drone_tf',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    output='screen',
                    parameters=[{
                        **common_parameters, 
                        'frame_id': 'base_link',
                        'map_frame_id': 'map',
                        'odom_frame_id': 'odom',
                        'publish_tf': True,
                        'odom_sensor_sync': True,
                        'subscribe_imu': True,
                        'wait_imu_to_init': True
                    }],
                    remappings=[
                        ('rgb/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                        ('rgb/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                        ('depth/image', '/depth_camera'),
                        ('imu', '/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
                        ('odom', '/odom'),
                    ]
                    ),
                Node(
                    package='rtabmap_viz',
                    executable='rtabmap_viz',
                    output='screen',
                    parameters=[{
                        **common_parameters,
                        'frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'subscribe_imu': True,
                        'sync_queue_size': 200,  # Much larger for Gazebo
                        'topic_queue_size': 30,
                        'approx_sync_max_interval': 0.1,  # Very permissive for visualization
                    }],
                    remappings=[
                        ('rgb/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                        ('rgb/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                        ('depth/image', '/depth_camera'),
                        ('imu', '/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
                        ('odom', '/odom'),
                    ]
                    ),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    parameters=[{
                        **common_parameters
                    }],
                    arguments=['-d', os.path.join(
                        get_package_share_directory('rtabmap_rviz_plugins'), 'launch', 'rtabmap.rviz')],
                    output='screen'
                )
            ]
            )])


