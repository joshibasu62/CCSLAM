#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    px4_dir = os.path.join(os.getenv("HOME"), "PX4-Autopilot")
    
    # Path to the file containing the URDF you provided (both drones in one file)
    urdf_file = "/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/two_drones.urdf"

    # -------------------------------------------------------------------------
    # VSLAM Parameters (Specific to Drone 0)
    # -------------------------------------------------------------------------
    vslam_params_0 = {
        'use_sim_time': True,
        'frame_id': 'x500_drone_0/base_link',
        'map_frame_id': 'x500_drone_0/map',
        'odom_frame_id': 'x500_drone_0/odom',
        
        'subscribe_rgbd': True,
        'subscribe_depth': False,      # We use rgbd_sync
        'subscribe_imu': True,

        'approx_sync': False,          # Sync is done by rgbd_sync
        'queue_size': 200,
        'sync_queue_size': 100,
        
        'Odom/ResetCountdown': '1',     
        'Vis/MinInliers': '10',         
        'Odom/Strategy': '0',           
        'wait_for_transform': 0.2,
        'Optimizer/GravitySigma': '0.3',
        'wait_imu_to_init': True,
        'publish_tf': True,

        'Grid/3D': True,
        'Grid/RayTracing': True,
        'Grid/MaxGroundHeight': '0.1', 
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '2',
        
        'database_path': '~/.ros/rtabmap_drone_0.db'
    }

    # -------------------------------------------------------------------------
    # VSLAM Parameters (Specific to Drone 1)
    # -------------------------------------------------------------------------
    vslam_params_1 = {
        'use_sim_time': True,
        'frame_id': 'x500_drone_1/base_link',
        'map_frame_id': 'x500_drone_1/map',
        'odom_frame_id': 'x500_drone_1/odom',
        
        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_imu': True,

        'approx_sync': False,
        'queue_size': 200,
        'sync_queue_size': 100,
        
        'Odom/ResetCountdown': '1',     
        'Vis/MinInliers': '10',         
        'Odom/Strategy': '0',           
        'wait_for_transform': 0.2,
        'Optimizer/GravitySigma': '0.3',
        'wait_imu_to_init': True,
        'publish_tf': True,

        'Grid/3D': True,
        'Grid/RayTracing': True,
        'Grid/MaxGroundHeight': '0.1', 
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '2',
        
        'database_path': '~/.ros/rtabmap_drone_1.db'
    }

    return LaunchDescription([
        # 1. Start Gazebo + Drone 0 (PX4)
        ExecuteProcess(
            cmd=["gnome-terminal", "--", "make", "-C", px4_dir, "px4_sitl", "gz_x500_depth"],
            output="screen",
            shell=True
        ),

        # 2. QGroundControl
        ExecuteProcess(
            cmd=["gnome-terminal", "--", "./QGroundControl-x86_64.AppImage"],
            cwd=os.path.expanduser("~/Downloads"),
            output="screen",
            shell=True
        ),

        # 3. MicroXRCEAgent
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "--port", "8888"],
            output="screen",
        ),

        # 4. Start Drone 1 (PX4) - Delayed
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "gnome-terminal", "--", "bash", "-c",
                        "cd " + px4_dir +
                        " && PX4_SYS_AUTOSTART=4001 "
                        'PX4_GZ_MODEL_POSE="0,-8,0" '
                        'PX4_GZ_MODEL_ORIENTATION="0,0,1.5708" '
                        "PX4_SIM_MODEL=gz_x500_depth "
                        "./build/px4_sitl_default/bin/px4 -i 1; exec bash"
                    ],
                    output="screen",
                )
            ],
        ),

        # 5. Bring up SLAM Infrastructure
        TimerAction(
            period=20.0,
            actions=[
                # A. Global Robot State Publisher (Loads your combined URDF)
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher_2drones",
                    output="screen",
                    parameters=[
                        {"robot_description": open(urdf_file).read()},
                        {"use_sim_time": True},
                    ],
                ),

                # B. Static TF: World -> Maps (Set initial pose offsets here)
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_to_d0_map",
                    arguments=["0", "0", "0", "0", "0", "0", "world", "x500_drone_0/map"],
                    output="screen",
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_to_d1_map",
                    arguments=["0", "-8", "0", "0", "0", "1.5708", "world", "x500_drone_1/map"],
                    output="screen",
                ),

                # C. ROS GZ Bridge (Handles topics for BOTH drones)
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="gz_bridge_rgbd_imu_2drones",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                    arguments=[
                        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        
                        # Drone 0 Raw Topics
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",

                        # Drone 1 Raw Topics
                        "/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        "/world/default/model/x500_depth_1/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_1/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    ],
                    remappings=[
                        # Drone 0 Remappings
                        ("/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image", "/x500_drone_0/rgb/image"),
                        ("/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info", "/x500_drone_0/rgb/camera_info"),
                        ("/world/default/model/x500_depth_0/link/camera_link/sensor/StereoOV7251/depth_image", "/x500_drone_0/depth/image"),
                        ("/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu", "/x500_drone_0/imu/data"),

                        # Drone 1 Remappings
                        ("/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image", "/x500_drone_1/rgb/image"),
                        ("/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info", "/x500_drone_1/rgb/camera_info"),
                        ("/world/default/model/x500_depth_1/link/camera_link/sensor/StereoOV7251/depth_image", "/x500_drone_1/depth/image"),
                        ("/world/default/model/x500_depth_1/link/base_link/sensor/imu_sensor/imu", "/x500_drone_1/imu/data"),
                    ],
                ),

                # ==========================================================
                # DRONE 0 NODES
                # ==========================================================
                Node(
                    package="rtabmap_sync",
                    executable="rgbd_sync",
                    name="rgbd_sync",
                    namespace="x500_drone_0",
                    output="screen",
                    parameters=[{
                        "use_sim_time": True,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.04,
                        "queue_size": 200,
                        "sync_queue_size": 100,
                    }],
                    remappings=[
                        ("rgb/image", "/x500_drone_0/rgb/image"),
                        ("rgb/camera_info", "/x500_drone_0/rgb/camera_info"),
                        ("depth/image", "/x500_drone_0/depth/image"),
                    ],
                ),
                Node(
                    package="rtabmap_odom",
                    executable="rgbd_odometry",
                    name="rgbd_odometry",
                    namespace="x500_drone_0",
                    output="screen",
                    parameters=[vslam_params_0],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                    ],
                ),
                Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap",
                    namespace="x500_drone_0",
                    output="screen",
                    parameters=[vslam_params_0],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ("odom", "/x500_drone_0/odom"),
                    ],
                    arguments=["-d"],
                ),
                Node(
                    package="rtabmap_viz",
                    executable="rtabmap_viz",
                    name="rtabmap_viz",
                    namespace="x500_drone_0",
                    output="screen",
                    parameters=[vslam_params_0],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ("odom", "/x500_drone_0/odom"),
                    ],
                ),

                # ==========================================================
                # DRONE 1 NODES
                # ==========================================================
                Node(
                    package="rtabmap_sync",
                    executable="rgbd_sync",
                    name="rgbd_sync",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[{
                        "use_sim_time": True,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.04,
                        "queue_size": 200,
                        "sync_queue_size": 100,
                    }],
                    remappings=[
                        ("rgb/image", "/x500_drone_1/rgb/image"),
                        ("rgb/camera_info", "/x500_drone_1/rgb/camera_info"),
                        ("depth/image", "/x500_drone_1/depth/image"),
                    ],
                ),
                Node(
                    package="rtabmap_odom",
                    executable="rgbd_odometry",
                    name="rgbd_odometry",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[vslam_params_1],
                    remappings=[
                        ("imu", "/x500_drone_1/imu/data"),
                    ],
                ),
                Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[vslam_params_1],
                    remappings=[
                        ("imu", "/x500_drone_1/imu/data"),
                        ("odom", "/x500_drone_1/odom"),
                    ],
                    arguments=["-d"],
                ),
                Node(
                    package="rtabmap_viz",
                    executable="rtabmap_viz",
                    name="rtabmap_viz",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[vslam_params_1],
                    remappings=[
                        ("imu", "/x500_drone_1/imu/data"),
                        ("odom", "/x500_drone_1/odom"),
                    ],
                ),

                # ==========================================================
                # GLOBAL RVIZ
                # ==========================================================
                Node(
                    package="rviz2",
                    executable="rviz2",
                    output="screen",
                    arguments=["-d", os.path.join(
                        get_package_share_directory("rtabmap_rviz_plugins"),
                        "launch",
                        "rtabmap.rviz"
                    )],
                    parameters=[{"use_sim_time": True}],
                ),
            ],
        ),
    ])