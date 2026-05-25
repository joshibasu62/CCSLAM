from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    IMAGE_TOPIC = '/camera/rgb/image_raw'             
    INFO_TOPIC = '/camera/rgb/camera_info'
    STEREO_TOPIC = '/camera/stereo/image_raw'

    return LaunchDescription([


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