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

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch',
         glob.glob('launch/*.launch.py')),

        # Models (KEEP STRUCTURE)
        *[
            (
                os.path.join('share', package_name, os.path.dirname(f)),
                [f]
            )
            for f in model_files
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='himanshu',
    maintainer_email='078bme014.himanshu@pcampus.edu.np',
    description='Drone SLAM package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'odom_drone_tf = drone_slam_pkg.odom_drone_tf:main',
            'odom_drone_tf_1 = drone_slam_pkg.odom_drone_tf_1:main',
        ],
    },
)
