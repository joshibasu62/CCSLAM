#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

   
    px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
    
    
    urdf_file_path = os.path.join(
        get_package_share_directory('drone_slam_pkg'),
        'model/x500_depth',
        'x500_depth.sdf' 
    )

    
    rtabmap_common_params = [
        {'use_sim_time': True},
        {'subscribe_depth': True},
        {'subscribe_rgbd': True},  # Enable RGB-D synchronization
        {'subscribe_odom_info': False},
        {'approx_sync': True},
        {'approx_sync_max_interval': 0.05},
        {'wait_imu_to_init': True},
        {'queue_size': 50} # Increase queue size for potentially high-rate sim data
    ]

  
   
    start_px4_gazebo_sim = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth_lawn'],
        output='screen',
        shell=True
    )

    start_qgroundcontrol = ExecuteProcess(
        cmd=['gnome-terminal', '--', os.path.expanduser('~/Downloads/QGroundControl.AppImage')],
        output='screen'
    )

    start_microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '--port', '8888'],
        output='screen'
    )

    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/lawn/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/lawn/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )
    px4_imu_bridge = Node(
        package='px4_ros_bridge',
        executable='px4_imu_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    px4_odom_bridge = Node(
        package='px4_ros_bridge',
        executable='px4_odom_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file_path]
    )


    
    rtabmap_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=rtabmap_common_params + [{
            'frame_id': 'base_link',         
            'odom_frame_id': 'odom',          
            'publish_tf': True,               
            'args': '--delete_db_on_start'
        }],
        remappings=[
            ('rgb/image', '/world/lawn/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
            ('depth/image', '/depth_camera'),
            ('rgb/camera_info', '/world/lawn/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
            ('imu', '/fmu/out/imu') 
        ],
        name='rtabmap_odometry'
    )

    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=rtabmap_common_params + [{
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'publish_tf': True,               
            'odom_sensor_sync': True,
        }],
        remappings=[
            ('rgb/image', '/world/lawn/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
            ('depth/image', '/depth_camera'),
            ('rgb/camera_info', '/world/lawn/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
            ('imu', '/fmu/out/imu'),
            ('odom', '/odom') 
        ],
        name='rtabmap_slam'
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=rtabmap_common_params,
        remappings=[
            ('rgb/image', '/world/lawn/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
            ('depth/image', '/depth_camera'),
            ('rgb/camera_info', '/world/lawn/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
            ('odom', '/odom')
        ],
        name='rtabmap_viz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    
    return LaunchDescription([
       
        start_px4_gazebo_sim,
        start_qgroundcontrol,
        start_microxrce_agent,

        gz_ros_bridge,
        px4_imu_bridge,
        px4_odom_bridge, 
        robot_state_publisher_node,

        rtabmap_odometry_node,
        rtabmap_slam_node,
        rtabmap_viz_node,
        
        rviz_node
    ])
