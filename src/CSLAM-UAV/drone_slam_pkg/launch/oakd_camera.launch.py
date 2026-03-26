from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    # CAMERA_BASE_FRAME = 'camera_rgb_camera_optical_frame'       
    IMAGE_TOPIC = '/camera/rgb/image_raw'             # from: ros2 topic list
    INFO_TOPIC = '/camera/rgb/camera_info'
    STEREO_TOPIC = '/camera/stereo/image_raw'

    return LaunchDescription([

        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_camera_tf',
        #     arguments=['0.12', '0.03', '0.242', '0', '0', '0',
        #                 'x500_drone_1/base_link', 'x500_drone_1/camera_link'],
        # ),

        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_link_to_oak',
        #     arguments=['0', '0', '0', '0', '0', '0',
        #                 'x500_drone_1/camera_link', CAMERA_BASE_FRAME],
        # ),

        # OAK-D Camera
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[os.path.expanduser('~/oak_run1.yaml')],
            remappings=[
                (IMAGE_TOPIC,  '/x500_drone_1/color/image_raw'),
                (INFO_TOPIC,   '/x500_drone_1/color/camera_info'),
                (STEREO_TOPIC, '/x500_drone_1/stereo/image_raw'),
            ]
        ),
    ])