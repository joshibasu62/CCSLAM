from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'

    return LaunchDescription([
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
                "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",

                # Camera info
                "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",

                # Depth camera raw image
                "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",

                # Depth camera as PointCloud2
                "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",

                # IMU
                "/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()},
                        {'use_sim_time': True}
                        ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rtabmap_odom',
            output='screen',
            parameters=[
                {'use_sim_time': True},

                {'frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'publish_tf': True},

                {'subscribe_rgbd': False},
                {'subscribe_depth': True},
                {'approx_sync': True},

                {'subscribe_imu': True},
                {'wait_imu_to_init': True}
            ],
            remappings=[

                ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/depth_camera'),
                ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
            ]
        ),

        # Node(
        #     package='drone_slam_pkg',
        #     executable='odom_drone_tf',
        #     name='odom_drone_tf',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                {'use_sim_time': True},

                # TF frames
                {'frame_id': 'base_link'},   # robot base frame
                {'odom_frame_id': 'odom'},   # matches rtabmap_odom
                {'map_frame_id': 'map'},     # global SLAM frame

                # Sensor subscriptions
                {'subscribe_rgbd': False},
                {'subscribe_depth': True},
                {'approx_sync': True},

                # # Optional: use IMU in SLAM as well
                # {'subscribe_imu': True},
                # {'wait_imu_to_init': True},

                # Optional: delete or set your own database path
                # {'database_path': '/home/basanta-joshi/.ros/rtabmap.db'},
            ],
            # remappings=[
            #     ('rgb/image',       '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
            #     ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
            #     ('depth/image',     '/depth_camera'),
            #     # ('odom','/odom')
            #     ('imu',             '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
            #     # By default, rtabmap subscribes to "odom" topic and TF; no remap needed
            # ]
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
            ]
            # Default topic names already match rtabmap_slam; no remaps needed
        ),

    ])

