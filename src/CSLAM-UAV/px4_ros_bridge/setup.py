from setuptools import find_packages, setup
import glob
package_name = 'px4_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='himanshu',
    maintainer_email='078bme014.himanshu@pcampus.edu.np',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'px4_imu_bridge = px4_ros_bridge.px4_imu_bridge_node:main',
                    'px4_odom_bridge = px4_ros_bridge.px4_odom_bridge_node:main'
        ],
    },
)
