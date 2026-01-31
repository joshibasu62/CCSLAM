



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

    # vslam_params = {
    #     'use_sim_time': True,
    #     'frame_id': 'base_link',
    #     'map_frame_id': 'map',
    #     'odom_frame_id': 'odom',

    #     # Sensor subscriptions
    #     'subscribe_rgbd': True,
    #     'subscribe_depth': False,     
    #     'subscribe_imu': True,

    #     # Sync/queues
    #     'approx_sync': False,          # IMPORTANT: sync is done by rgbd_sync
    #     'queue_size': 200,
    #     'sync_queue_size': 100,

        
    #     'Odom/ResetCountdown': '1',     # <--- CRITICAL: If lost, reset immediately
    #     'Vis/MinInliers': '20',         # <--- CRITICAL: Track with fewer points
    #     'Odom/Strategy': '0',           # 0=Frame-to-Frame (Faster for drones)
    #     'wait_for_transform': 0.2,
    #     'Optimizer/GravitySigma': '0.3',

    #     'wait_imu_to_init': True,
    #     'publish_tf': True,


    #     #for 2d maps
    #     # 'Grid/3D': True,
    #     # 'Grid/RayTracing': True,
    #     # 'Grid/MaxGroundHeight': '0.1', 
    #     # 'Grid/MaxObstacleHeight': '2.0',
    #     # 'Grid/NoiseFilteringRadius': '0.05',
    #     # 'Grid/NoiseFilteringMinNeighbors': '2',
    # }
    vslam_params = {
        'use_sim_time': True,

        # --- TF / Frames ---
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'publish_tf': True,

        # --- Subscription ---
        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_imu': True,
        
        # --- Synchronization ---
        'approx_sync': False, # False because rgbd_sync does the work
        'queue_size': 20,
        'wait_imu_to_init': True,

        # ====================================================
        # 1. PARAMETERS TO GENERATE THE 2D MAP (LIKE IMAGE)
        # ====================================================
        'Grid/3D': 'False',               # True=3D Octomap, False=2D Occupancy Grid (What you want)
        'Grid/RayTracing': 'True',        # CLEANS THE MAP. Clears empty space when robot moves.
        'Grid/FromDepth': 'True',         # Create 2D map from the depth camera
        'Grid/MaxGroundHeight': '0.2',    # Points below 20cm are "floor" (white)
        'Grid/MaxObstacleHeight': '2.0',  # Points above 2m are ignored (don't map the ceiling)
        'Grid/NormalsSegmentation': 'False', # Faster for 2D maps
        
        # This forces the map to stay flat on the ground (2D SLAM mode)
        # Crucial for drones if you only care about X/Y navigation
        'Reg/Force3DoF': 'True',          

        # ====================================================
        # 2. PARAMETERS TO REDUCE DRIFT
        # ====================================================
        
        # Visual Odometry Strategy
        'Odom/Strategy': '0',             # 0=F2F (Frame-to-Frame). Best for drones (fast movement).
        'Odom/ResetCountdown': '1',       # Reset immediately if tracking is lost
        'Odom/Holonomic': 'False',        # False for quadcopters (they drift sideways, but this assumes diff drive logic usually. Safe as False or True usually)
        
        # Feature Extraction (Drift Killer)
        'Vis/FeatureType': '2',           # 2=GFTT (Good Features To Track). Better than default FAST.
        'Vis/MaxFeatures': '1000',        # Track MORE points (Default is 1000, ensure it's not lower)
        'Vis/MinInliers': '20',           # Require at least 15 good points to count as "moving"
        'Vis/EstimationType': '1',        # 1=PnP (Perspective-n-Point). More robust estimation.

        # IMU Fusion (Gravity Alignment)
        'Optimizer/GravitySigma': '0.05', # Very Low = Trust IMU Gravity A LOT. Prevents map tilting.
        
        # Loop Closure (Map Correction)
        'RGBD/OptimizeFromGraphEnd': 'True', # Optimize graph from the newest node
        'RGBD/AngularUpdate': '0.05',     # Update map on small rotations (5 degrees)
        'RGBD/LinearUpdate': '0.05',      # Update map on small movements (5 cm)
        
        # Loop Closure Detection
        'Kp/DetectorStrategy': '2',       # 2=GFTT. Matches Vis/FeatureType.
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
            period=1.0,
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
                    ],
                    remappings=[
                        # Drone 0
                    
                        ("/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu", "/x500_drone_0/imu/data"),
                    ],
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
                


                # Node(
                #     package='drone_slam_pkg',
                #     executable='odom_drone_tf_single',
                #     name='odom_drone_tf_single',
                #     output='screen',
                #     parameters=[{'use_sim_time': True}]
                # ),
                Node(
                    package='rtabmap_sync',
                    executable='rgbd_sync',
                    name='rgbd_sync',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'approx_sync': True,                # sync RGB+Depth here
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

                # 2) Visual odometry (publishes TF rtabmap_odom -> base_link and topic /rtabmap/odom)
                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        # rgbd_image is already /rtabmap/rgbd_image from the rgbd_sync node
                    ],
                ),

                # 3) SLAM (publishes map -> rtabmap_odom TF)
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
                    ],
                    arguments=['-d'],   # delete previous ~/.ros/rtabmap.db (same behavior as many examples)
                ),

                # 4) Visualization
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

                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', os.path.join(
                        get_package_share_directory('rtabmap_rviz_plugins'),
                        'launch', 'rtabmap.rviz'
                    )],
                    parameters=[{'use_sim_time': True}]
                ),
            ]
            )])


