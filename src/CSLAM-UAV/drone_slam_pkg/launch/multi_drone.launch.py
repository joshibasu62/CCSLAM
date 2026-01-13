#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    px4_dir = os.path.join(os.getenv("HOME"), "PX4-Autopilot")

    # Your 2-drone URDF (contains x500_drone_0/* and x500_drone_1/* links/joints)
    urdf_file = "/home/basanta-joshi/Desktop/cslam/src/CSLAM-UAV/drone_slam_pkg/urdf/two_drones.urdf"

    world_name = "default"
    gz_model_0 = "x500_depth_0"
    gz_model_1 = "x500_depth_1"

    drone0 = "x500_drone_0"
    drone1 = "x500_drone_1"

    # ---------- RTAB-Map params (per drone, written explicitly) ----------
    vslam_params_0 = {
        "use_sim_time": True,
        "frame_id": f"{drone0}/base_link",
        "map_frame_id": f"{drone0}/map",
        "odom_frame_id": f"{drone0}/odom",
        "subscribe_rgbd": True,
        "subscribe_depth": False,
        "subscribe_imu": True,
        "approx_sync": False,     # sync done in rgbd_sync
        "queue_size": 200,
        "sync_queue_size": 100,
        "wait_imu_to_init": True,
        "publish_tf": True,
    }

    vslam_params_1 = {
        "use_sim_time": True,
        "frame_id": f"{drone1}/base_link",
        "map_frame_id": f"{drone1}/map",
        "odom_frame_id": f"{drone1}/odom",
        "subscribe_rgbd": True,
        "subscribe_depth": False,
        "subscribe_imu": True,
        "approx_sync": False,     # sync done in rgbd_sync
        "queue_size": 200,
        "sync_queue_size": 100,
        "wait_imu_to_init": True,
        "publish_tf": True,
    }

    # ---------- Short ROS topic names ----------
    d0_rgb = f"/{drone0}/rgb/image"
    d0_info = f"/{drone0}/rgb/camera_info"
    d0_depth = f"/{drone0}/depth/image"
    d0_imu = f"/{drone0}/imu/data"

    d1_rgb = f"/{drone1}/rgb/image"
    d1_info = f"/{drone1}/rgb/camera_info"
    d1_depth = f"/{drone1}/depth/image"
    d1_imu = f"/{drone1}/imu/data"

    # ---------- Gazebo topics ----------
    gz0_rgb = f"/world/{world_name}/model/{gz_model_0}/link/camera_link/sensor/IMX214/image"
    gz0_info = f"/world/{world_name}/model/{gz_model_0}/link/camera_link/sensor/IMX214/camera_info"
    gz0_depth = f"/world/{world_name}/model/{gz_model_0}/link/camera_link/sensor/StereoOV7251/depth_image"
    gz0_imu = f"/world/{world_name}/model/{gz_model_0}/link/base_link/sensor/imu_sensor/imu"

    gz1_rgb = f"/world/{world_name}/model/{gz_model_1}/link/camera_link/sensor/IMX214/image"
    gz1_info = f"/world/{world_name}/model/{gz_model_1}/link/camera_link/sensor/IMX214/camera_info"
    gz1_depth = f"/world/{world_name}/model/{gz_model_1}/link/camera_link/sensor/StereoOV7251/depth_image"
    gz1_imu = f"/world/{world_name}/model/{gz_model_1}/link/base_link/sensor/imu_sensor/imu"

    return LaunchDescription([
        # PX4 + Gazebo (first instance)
        ExecuteProcess(
            cmd=["gnome-terminal", "--", "make", "-C", px4_dir, "px4_sitl", "gz_x500_depth"],
            output="screen",
        ),

        # QGC
        ExecuteProcess(
            cmd=["./QGroundControl-x86_64.AppImage"],
            cwd=os.path.expanduser("~/Downloads"),
            output="screen",
        ),

        # XRCE Agent
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "--port", "8888"],
            output="screen",
        ),

        # PX4 second instance (spawns drone 1)
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

        # Bring up bridges + TF + RTAB-Map after sim is ready
        TimerAction(
            period=20.0,
            actions=[
                # Bridge clock + both drones sensors
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="gz_bridge_rgbd_imu_2drones",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                    arguments=[
                        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

                        f"{gz0_rgb}@sensor_msgs/msg/Image[gz.msgs.Image",
                        f"{gz0_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        f"{gz0_depth}@sensor_msgs/msg/Image[gz.msgs.Image",
                        f"{gz0_imu}@sensor_msgs/msg/Imu[gz.msgs.IMU",

                        f"{gz1_rgb}@sensor_msgs/msg/Image[gz.msgs.Image",
                        f"{gz1_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        f"{gz1_depth}@sensor_msgs/msg/Image[gz.msgs.Image",
                        f"{gz1_imu}@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    ],
                    remappings=[
                        (gz0_rgb, d0_rgb),
                        (gz0_info, d0_info),
                        (gz0_depth, d0_depth),
                        (gz0_imu, d0_imu),

                        (gz1_rgb, d1_rgb),
                        (gz1_info, d1_info),
                        (gz1_depth, d1_depth),
                        (gz1_imu, d1_imu),
                    ],
                ),

                # Publish fixed sensor->base transforms for BOTH drones from your URDF
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

                # ---- Static TF anchors (world -> each drone odom) ----
                # Adjust these to your actual spawn poses if needed.
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_to_x500_drone_0_odom",
                    arguments=["0", "0", "0", "0", "0", "0", "world", f"{drone0}/map"],
                    output="screen",
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_to_x500_drone_1_odom",
                    arguments=["0", "-8", "0", "0", "0", "1.5708", "world", f"{drone1}/map"],
                    output="screen",
                ),

                #drone0

                # Node(
                #     package='drone_slam_pkg',
                #     executable='odom_drone_tf',
                #     name='odom_drone_tf',
                #     output='screen',
                #     parameters=[{'use_sim_time': True}]
                # ),
                
                Node(
                    package="rtabmap_sync",
                    executable="rgbd_sync",
                    name="rgbd_sync",
                    namespace=drone0,
                    output="screen",
                    parameters=[{
                        "use_sim_time": True,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.04,
                        "queue_size": 200,
                        "sync_queue_size": 100,
                    }],
                    remappings=[
                        ("rgb/image", d0_rgb),
                        ("rgb/camera_info", d0_info),
                        ("depth/image", d0_depth),
                    ],
                ),

                Node(
                    package="rtabmap_odom",
                    executable="rgbd_odometry",
                    name="rgbd_odometry",
                    namespace=drone0,
                    output="screen",
                    parameters=[vslam_params_0],
                    remappings=[
                        ("imu", d0_imu),
                    ],
                ),

                Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap",
                    namespace=drone0,
                    output="screen",
                    parameters=[{
                        **vslam_params_0,
                        "database_path": f"~/.ros/rtabmap_{drone0}.db",
                    }],
                    remappings=[
                        ("imu", d0_imu),
                        ("odom", f"/{drone0}/odom"),
                    ],
                    arguments=["-d"],
                ),

                Node(
                    package="rtabmap_viz",
                    executable="rtabmap_viz",
                    name="rtabmap_viz",
                    namespace=drone0,
                    output="screen",
                    parameters=[vslam_params_0],
                    remappings=[
                        ("imu", d0_imu),
                        ("odom", f"/{drone0}/odom"),
                    ],
                ),

                #drone1
                # Node(
                #     package='drone_slam_pkg',
                #     executable='odom_drone_tf_1',
                #     name='odom_drone_tf_1',
                #     output='screen',
                #     parameters=[{'use_sim_time': True}]
                # ),

                Node(
                    package="rtabmap_sync",
                    executable="rgbd_sync",
                    name="rgbd_sync",
                    namespace=drone1,
                    output="screen",
                    parameters=[{
                        "use_sim_time": True,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.04,
                        "queue_size": 200,
                        "sync_queue_size": 100,
                    }],
                    remappings=[
                        ("rgb/image", d1_rgb),
                        ("rgb/camera_info", d1_info),
                        ("depth/image", d1_depth),
                    ],
                ),

                Node(
                    package="rtabmap_odom",
                    executable="rgbd_odometry",
                    name="rgbd_odometry",
                    namespace=drone1,
                    output="screen",
                    parameters=[vslam_params_1],
                    remappings=[
                        ("imu", d1_imu),
                    ],
                ),
                

                Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap",
                    namespace=drone1,
                    output="screen",
                    parameters=[{
                        **vslam_params_1,
                        "database_path": f"~/.ros/rtabmap_{drone1}.db",
                    }],
                    remappings=[
                        ("imu", d1_imu),
                        ("odom", f"/{drone1}/odom"),
                    ],
                    arguments=["-d"],
                ),

                Node(
                    package="rtabmap_viz",
                    executable="rtabmap_viz",
                    name="rtabmap_viz",
                    namespace=drone1,
                    output="screen",
                    parameters=[vslam_params_1],
                    remappings=[
                        ("imu", d1_imu),
                        ("odom", f"/{drone1}/odom"),
                    ],
                ),

                # RViz (single)
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