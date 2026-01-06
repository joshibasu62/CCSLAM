#!/usr/bin/env python3

# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, TimerAction
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():

#     px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
#     urdf_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'

#     # -------------------------------
#     # Common RTAB-Map parameters
#     # -------------------------------
#     common_parameters = {
#         'subscribe_depth': True,
#         'subscribe_rgbd': False,
#         'subscribe_stereo': False,
#         'subscribe_odom_info': False,
#         'approx_sync': True,
#         'subscribe_imu': True,
#         'use_sim_time': True,
#         'always_check_imu_tf': False,
#         'load_db': False,
#         'wait_imu_to_init': True,
#         'approx_sync_max_interval': 0.04,
#         'queue_size': 200,
#         'sync_queue_size': 100,
#         'topic_queue_size': 20,
#     }

#     # Read URDF safely
#     with open(urdf_file, 'r') as f:
#         robot_description = f.read()

#     return LaunchDescription([

#         # ---------------------------------
#         # Micro XRCE Agent (PX4 ↔ ROS2)
#         # ---------------------------------
#         ExecuteProcess(
#             cmd=['MicroXRCEAgent', 'udp4', '--port', '8888'],
#             output='screen'
#         ),

#         # ---------------------------------
#         # Delayed startup (Gazebo ready)
#         # ---------------------------------
#         TimerAction(
#             period=5.0,
#             actions=[

#                 # ---------------------------------
#                 # Gazebo ↔ ROS bridge
#                 # ---------------------------------
#                 Node(
#                     package='ros_gz_bridge',
#                     executable='parameter_bridge',
#                     name='gz_bridge_rgbd_imu',
#                     output='screen',
#                     parameters=[{'use_sim_time': True}],
#                     arguments=[
#                         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
#                         "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
#                         "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
#                         "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
#                         "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
#                         "/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
#                     ]
#                 ),

#                 # ---------------------------------
#                 # Robot State Publisher
#                 # ---------------------------------
#                 Node(
#                     package='robot_state_publisher',
#                     executable='robot_state_publisher',
#                     name='robot_state_publisher',
#                     output='screen',
#                     parameters=[
#                         {'robot_description': robot_description},
#                         {'use_sim_time': True}
#                     ]
#                 ),

#                 # ---------------------------------
#                 # RTAB-Map RGBD Odometry
#                 # ---------------------------------
#                 Node(
#                     package='rtabmap_odom',
#                     executable='rgbd_odometry',
#                     output='screen',
#                     parameters=[{
#                         **common_parameters,
#                         'frame_id': 'base_link',
#                         'odom_frame_id': 'odom',
#                         'publish_tf': True
#                     }],
#                     remappings=[
#                         ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
#                         ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
#                         ('depth/image', '/depth_camera'),
#                         ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
#                     ]
#                 ),

#                 # ---------------------------------
#                 # RTAB-Map SLAM
#                 # ---------------------------------
#                 Node(
#                     package='rtabmap_slam',
#                     executable='rtabmap',
#                     output='screen',
#                     parameters=[{
#                         **common_parameters,
#                         'frame_id': 'base_link',
#                         'map_frame_id': 'map',
#                         'odom_frame_id': 'odom',
#                         'publish_tf': True,
#                         'odom_sensor_sync': True,
#                         'subscribe_imu': True,
#                         'wait_imu_to_init': True
#                     }],
#                     remappings=[
#                         ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
#                         ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
#                         ('depth/image', '/depth_camera'),
#                         ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
#                         ('odom', '/odom'),
#                     ]
#                 ),

#                 # ---------------------------------
#                 # RTAB-Map Visualization
#                 # ---------------------------------
#                 Node(
#                     package='rtabmap_viz',
#                     executable='rtabmap_viz',
#                     output='screen',
#                     parameters=[{
#                         **common_parameters,
#                         'frame_id': 'base_link',
#                         'map_frame_id': 'map',
#                         'odom_frame_id': 'odom',
#                         'sync_queue_size': 200,
#                         'topic_queue_size': 30,
#                         'approx_sync_max_interval': 0.1,
#                     }],
#                     remappings=[
#                         ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
#                         ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
#                         ('depth/image', '/depth_camera'),
#                         ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
#                         ('odom', '/odom'),
#                     ]
#                 ),

#                 # ---------------------------------
#                 # RViz
#                 # ---------------------------------
#                 Node(
#                     package='rviz2',
#                     executable='rviz2',
#                     output='screen',
#                     parameters=[{'use_sim_time': True}],
#                     arguments=[
#                         '-d',
#                         os.path.join(
#                             get_package_share_directory('rtabmap_rviz_plugins'),
#                             'launch',
#                             'rtabmap.rviz'
#                         )
#                     ]
#                 ),
#             ]
#         )
#     ])




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
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--', 'make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth_baylands'],
        #     output='screen',
        #     shell=True
        # ),
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--', ' ./QGroundControl.AppImage'],
        #     cwd=os.path.expanduser('~/Desktop'),
        #     output='screen',
        #     shell=True
        # ),
        ExecuteProcess(
             cmd=['MicroXRCEAgent','udp4', '--port', '8888']
        ),
        TimerAction(
            period=5.0,
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
                #             ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                #             ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                #             ('depth/image', '/depth_camera'),
                #             ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
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
                        ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                        ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                        ('depth/image', '/depth_camera'),
                        ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
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
                        ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                        ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                        ('depth/image', '/depth_camera'),
                        ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
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


# #!/usr/bin/env python3
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.actions import LogInfo
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
#     urdf_file = '/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'
    
#     # Base parameters for all RTAB-Map nodes
#     common_parameters = {
#         'subscribe_depth': True,
#         'subscribe_rgbd': False,
#         'subscribe_stereo': False,
#         'subscribe_odom_info': False,
#         'approx_sync': True,
#         'subscribe_imu': True,
#         'use_sim_time': True,
#         'always_check_imu_tf': False,
#         'load_db': False,
#         'wait_imu_to_init': False,
#         'approx_sync_max_interval': 0.04,  # Increased from 0.02 to handle Gazebo delays
#         'queue_size': 100,
#         'sync_queue_size': 100,  # Increased synchronization buffer
#         'topic_queue_size': 20,  # Increased topic buffer
#     }
    
#     return LaunchDescription([
#         # MicroXRCE Agent
#         ExecuteProcess(
#             cmd=['MicroXRCEAgent', 'udp4', '--port', '8888'],
#             output='screen'
#         ),
        
#         TimerAction(
#             period=5.0,
#             actions=[
#                 # Bridge for Gazebo topics
#                 Node(
#                     package='ros_gz_bridge',
#                     executable='parameter_bridge',
#                     name='gz_bridge_rgbd_imu',
#                     output='screen',
#                     parameters=[{'use_sim_time': True}],
#                     arguments=[
#                         # Simulation clock
#                         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        
#                         # RGB camera
#                         "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        
#                         # Camera info
#                         "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        
#                         # Depth camera
#                         "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                        
#                         # Depth camera as PointCloud2
#                         "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                        
#                         # IMU
#                         "/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
#                     ]
#                 ),
                
#                 # Robot State Publisher
#                 Node(
#                     package='robot_state_publisher',
#                     executable='robot_state_publisher',
#                     name='robot_state_publisher',
#                     output='screen',
#                     parameters=[{
#                         'robot_description': open(urdf_file).read(),
#                         'use_sim_time': True
#                     }]
#                 ),
                
#                 # RTAB-Map Odometry Node
#                 Node(
#                     package='rtabmap_odom',
#                     executable='rgbd_odometry',
#                     name='rgbd_odometry',
#                     output='screen',
#                     parameters=[{
#                         **common_parameters,
#                         'frame_id': 'base_link',
#                         'odom_frame_id': 'odom',
#                         'publish_tf': True,
#                         'guess_min_translation': 0.0,
#                         'guess_min_rotation': 0.0,
#                         'odom_sensor_sync': True,
#                         'wait_for_transform': 0.2,
#                         'max_update_rate': 10.0,  # Hz
#                         'approx_sync_max_interval': 0.04,  # Allow 40ms difference
#                         'queue_size': 200,  # Larger queue for Gazebo
#                     }],
#                     remappings=[
#                         ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
#                         ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
#                         ('depth/image', '/depth_camera'),
#                         ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
#                     ]
#                 ),
                
#                 # RTAB-Map SLAM Node
#                 Node(
#                     package='rtabmap_slam',
#                     executable='rtabmap',
#                     name='rtabmap',
#                     output='screen',
#                     parameters=[{
#                         **common_parameters,
#                         'frame_id': 'base_link',
#                         'map_frame_id': 'map',
#                         'odom_frame_id': 'odom',
#                         'publish_tf': True,
#                         'odom_sensor_sync': True,
#                         'subscribe_imu': True,
#                         'wait_imu_to_init': True,
#                         'Vis/MaxFeatures': 1000,
#                         'Mem/STMSize': 30,
#                         'Mem/RehearsalSimilarity': 0.30,
#                         'Grid/CellSize': 0.05,
#                         'Grid/3D': True,
#                         'Grid/RayTracing': True,
#                         'Grid/GlobalMaxHeight': 4.0,
#                         'Grid/GlobalMinHeight': 0.0,
#                         'Grid/NormalsSegmentation': True,
#                         'Reg/Strategy': 1,  # 0=Visual, 1=ICP, 2=Vis+ICP
#                         'Icp/CorrespondenceRatio': 0.3,
#                         'Icp/Iterations': 10,
#                         'Icp/VoxelSize': 0.05,
#                         'Icp/PointToPlane': True,
#                         'Icp/PointToPlaneK': 20,
#                         'Icp/PointToPlaneRadius': 0.1,
#                         'Icp/MaxTranslation': 1.0,
#                         'Icp/MaxRotation': 0.5,
#                         'Icp/OutlierRatio': 0.7,
#                         'RGBD/OptimizeMaxError': 1.0,
#                         'RGBD/NeighborLinkRefining': True,
#                         'RGBD/ProximityBySpace': True,
#                         'RGBD/ProximityByTime': True,
#                         'RGBD/LocalRadius': 1.0,
#                         'RGBD/LocalMaxNeighbors': 10,
#                         'RGBD/LoopClosureReextractFeatures': False,
#                         'RGBD/AngularUpdate': 0.3,
#                         'RGBD/LinearUpdate': 0.1,
#                         'RGBD/OptimizeFromGraphEnd': False,
#                         'RGBD/OptimizeIterations': 10,
#                         'RGBD/ProximityPathMaxNeighbors': 1,
#                         'Kp/MaxFeatures': 1000,
#                         'Kp/DetectorStrategy': 0,
#                         'Kp/NNStrategy': 1,
#                         'Vis/FeatureType': 6,
#                         'Vis/CorNNType': 1,
#                         'Vis/CorNNDR': 0.8,
#                         'Vis/CorGuessWinSize': 20,
#                         'Vis/MaxFeatures': 1000,
#                         'Vis/MinInliers': 15,
#                         'Vis/EstimationType': 1,
#                         'Vis/PnPReprojError': 3.0,
#                         'Vis/PnPFlags': 0,
#                         'Vis/ForwardEstOnly': False,
#                         'Vis/Iterations': 200,
#                         'Vis/RefineIterations': 10,
#                         'Vis/MaxDepth': 4.0,
#                         'Vis/MinDepth': 0.0,
#                         'Vis/RoiRatios': '0.0 0.0 0.0 0.0',
#                         'Rtabmap/TimeThr': 700,
#                         'Rtabmap/MemoryThr': 0,
#                         'Rtabmap/DetectionRate': 1,
#                         'Rtabmap/ImagesAlreadyRectified': True,
#                         'Rtabmap/CreateIntermediateNodes': False,
#                         'Rtabmap/StatisticLogged': True,
#                         'Rtabmap/StatisticLoggedHeaders': True,
#                         'Rtabmap/LoopRatio': 0.0,
#                         'Rtabmap/LoopCovLimited': True,
#                         'Rtabmap/LoopThr': 0.0,
#                         'Rtabmap/ReextractFeatures': False,
#                         'Rtabmap/NewThr': 0.0,
#                         'Rtabmap/IncrementalMemory': True,
#                         'Rtabmap/IncrementalMemoryIO': True,
#                         'Rtabmap/Localization': False,
#                     }],
#                     remappings=[
#                         ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
#                         ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
#                         ('depth/image', '/depth_camera'),
#                         ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
#                         ('odom', '/odom'),
#                     ]
#                 ),
                
#                 # RTAB-Map Visualization Node
#                 Node(
#                     package='rtabmap_viz',
#                     executable='rtabmap_viz',
#                     name='rtabmap_viz',
#                     output='screen',
#                     parameters=[{
#                         **common_parameters,
#                         'frame_id': 'base_link',
#                         'odom_frame_id': 'odom',
#                         'subscribe_imu': True,
#                         'sync_queue_size': 200,  # Much larger for Gazebo
#                         'topic_queue_size': 30,
#                         'approx_sync_max_interval': 0.1,  # Very permissive for visualization
#                     }],
#                     remappings=[
#                         ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
#                         ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
#                         ('depth/image', '/depth_camera'),
#                         ('imu', '/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
#                         ('odom', '/odom'),
#                     ]
#                 ),
                
#                 # RViz2
#                 Node(
#                     package='rviz2',
#                     executable='rviz2',
#                     name='rviz2',
#                     parameters=[{'use_sim_time': True}],
#                     arguments=['-d', os.path.join(
#                         get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.rviz')],
#                     output='screen'
#                 ),
                
#                 # Diagnostic node to check topic rates (optional)
#                 # Node(
#                 #     package='diagnostic_aggregator',
#                 #     executable='aggregator_node',
#                 #     name='diagnostic_aggregator',
#                 #     parameters=[{'use_sim_time': True}],
#                 #     output='screen'
#                 # ),
#             ]
#         ),
        
#         # Log info about the launch
#         LogInfo(msg="RTAB-Map SLAM system starting with Gazebo simulation..."),
#         LogInfo(msg="Make sure Gazebo is running with the x500_depth model"),
#         LogInfo(msg="Topic rates will be checked automatically"),
#     ])