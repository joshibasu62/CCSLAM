#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
    urdf_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'

    vslam_params = {
        'use_sim_time': True,
        'frame_id': 'x500_drone_0/base_link',
        'map_frame_id': 'x500_drone_0/map',
        'odom_frame_id': 'x500_drone_0/odom',
        'guess_frame_id': 'x500_drone_0/base_link_stabilized',

        'subscribe_rgbd': True,
        'subscribe_depth': False,
        # 'subscribe_odom_info': True,
        'subscribe_odom': True,
        'subscribe_imu': True,
        'approx_sync': False,
        'queue_size': 200,
        'sync_queue_size': 100,

        # 'use_action_for_goal': True,

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

    # vslam_remappings = [
    #     ('imu', '/x500_drone_0/imu/data'),
    #     ('map', '/x500_drone_0/map'),
    # ]

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
                ("/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu", "/x500_drone_0/imu/data_raw"),
                ("/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image","/x500_drone_0/rgb/image"),
                ("/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info","/x500_drone_0/rgb/camera_info"),
                ("/depth_camera","/x500_drone_0/depth/image"),
            ],
        ),

        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_0/base_link', 'x500_depth_0/base_link/imu_sensor']),
        
        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0.12', '0.03', '0.242', '-1.570796327', '0', '-1.570796327', 'x500_drone_0/base_link', 'x500_drone_0/camera_link']),
        
        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0.0123', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_0/camera_link', 'x500_depth_0/camera_link/IMX214']),
        
        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0.01233', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_0/camera_link', 'x500_drone_0/camera_link/StereoOV7251']),


        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[
        #         {'robot_description': open(urdf_file).read()},
        #         {'use_sim_time': True},
        #         {'frame_prefix': 'x500_drone_0/'},
        #     ]
        # ),

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
                'x500_drone_0/base_link', 'x500_drone_0/middle'
            ],
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            namespace='x500_drone_0',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
                'use_sim_time': True,
            }],
            remappings=[
                ('imu/data_raw', '/x500_drone_0/imu/data_raw'),
                ('imu/data', '/x500_drone_0/imu/data'),
            ],
        ),

        Node(
            package='rtabmap_util',
            executable='imu_to_tf',
            namespace = 'x500_drone_0',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'fixed_frame_id': 'x500_drone_0/base_link_stabilized',
                'base_frame_id': 'x500_drone_0/base_link',
            }],
            remappings=[
                ('imu/data', '/x500_drone_0/imu/data'),
            ],
        ),

        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            namespace='x500_drone_0',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.04,
                'queue_size': 200,
                'sync_queue_size': 100,
            }],
            remappings=[
                ('rgb/image', '/x500_drone_0/rgb/image'),
                ('rgb/camera_info', '/x500_drone_0/rgb/camera_info'),
                ('depth/image', '/x500_drone_0/depth/image'),
            ],
        ),

        # Visual Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            namespace='x500_drone_0',
            output='screen',
            parameters=[vslam_params, {'odom_frame_id': 'x500_drone_0/odom'}],
            # remappings=vslam_remappings,
            remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ("map", "/x500_drone_0/map"),
                        ("odom", "/x500_drone_0/odom"), 
                    ],
            arguments=["--ros-args", "--log-level", 'info'],
        ),

        # SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='x500_drone_0',
            output='screen',
            parameters=[vslam_params],
            remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ("odom", "/x500_drone_0/odom"),
                        ("map", "/x500_drone_0/map"),
                    ],
            # remappings=vslam_remappings,
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
        
        #     remappings=vslam_remappings,
        # ),

        # Generates filtered point cloud for local costmap
        # Node(
        #     package='rtabmap_util',
        #     executable='point_cloud_xyz',
        #     output='screen',
        #     parameters=[{
        #         'decimation': 2,
        #         'max_depth': 3.0,
        #         'voxel_size': 0.02,
        #         'use_sim_time': True,
        #     }],
        #     remappings=[
        #         ('depth/image', '/x500_drone_0/depth/image'),
        #         ('depth/camera_info', '/x500_drone_0/rgb/camera_info'),
        #         ('cloud', '/camera/cloud'),
        #     ],
        # ),

        TimerAction(
            period=30.0,
            actions=[

                # Node(
                #     package='rtabmap_costmap_plugins',
                #     executable='voxel_marker',
                #     output='screen',
                #     namespace='local_costmap',
                #     parameters=[{'use_sim_time': True}],
                # ),

                # Node(
                #     package='px4_ros_com',
                #     executable='ros_odometry_to_vehicle_odometry',
                #     name='ros_odometry_to_vehicle_odometry',
                #     output='screen',
                #     parameters=[{
                #         'use_sim_time': True,
                #         'repeat_odom': False,
                #     }],
                #     remappings=[
                #         ('odom', '/x500_drone_0/odom'),
                #     ],
                # ),

                Node(
                    package='px4_ros_com',
                    executable='ros_odometry_to_vehicle_odometry',
                    name='ros_odometry_to_vehicle_odometry',
                    namespace='x500_drone_0',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'repeat_odom': False,
                        'map_frame_id': 'x500_drone_0/map',
                        'odom_topic': '/x500_drone_0/odom',
                        'vehicle_odometry_topic': '/fmu/in/vehicle_visual_odometry'
                    }],
                ),
            ]
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     arguments=['-d', '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/rviz/simulation.rviz'],
        #     parameters=[{'use_sim_time': True}]
        # ),

        # TimerAction(
        #     period=45.0,
        #     actions=[
        #         Node(
        #             package = 'px4_ros_com',
        #             executable = 'offboard_control',
        #             name = 'offboard_control',
        #             output = 'screen',
        #             parameters = [{
        #                 'use_sim_time': True,
        #             }],
        #         )
        #     ]
        # )
    ])