from setuptools import setup, find_packages
import glob
import os

package_name = 'drone_slam_pkg'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

model_files = package_files('urdf')
yaml_files = package_files('config')  
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        
        ('share/' + package_name + '/launch',
         glob.glob('launch/*.launch.py')),
         
        ('share/' + package_name + '/launch',
         glob.glob('launch/*.py')),
         

        *[
            (
                os.path.join('share', package_name, os.path.dirname(f)),
                [f]
            )
            for f in model_files
        ],
        *[
            (
                os.path.join('share', package_name, os.path.dirname(f)),
                [f]
            )
            for f in yaml_files
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joshibasu62',
    maintainer_email='joshibasu62@gmail.com',
    description='Drone SLAM package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'odom_drone_tf = drone_slam_pkg.odom_drone_tf:main',
            'odom_drone_tf_1 = drone_slam_pkg.odom_drone_tf_1:main',
            'odom_drone_tf_single = drone_slam_pkg.odom_drone_tf_single:main',
            'cloud_merger = drone_slam_pkg.cloud_merger:main',
            'px4_vel_bridge = drone_slam_pkg.px4_vel_bridge:main',
            'px4_imu_bridge = drone_slam_pkg.px4_imu_bridge:main',
            'visual_odom_converter = drone_slam_pkg.visual_odom_converter:main',
        ],
    },
)
