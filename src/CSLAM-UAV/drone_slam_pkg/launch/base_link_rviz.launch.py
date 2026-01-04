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
        # Node(
        #     package='drone_slam_pkg',
        #     executable='odom_drone_tf',
        #     name='odom_drone_tf',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rtabmap_odom',
            output='screen',
            parameters=[
                {'use_sim_time': True},

                # Frames
                {'frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'publish_tf': True},

                # RGB-D
                {'subscribe_rgbd': False},
                {'subscribe_depth': True},
                {'approx_sync': True},

                # IMU
                {'subscribe_imu': True},
                {'wait_imu_to_init': True}
            ],
            remappings=[
                # RGB camera
                ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),

                # Depth
                ('depth/image', '/depth_camera'),

                # IMU
                ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
            ]
        ),

    ])

