import os
from glob import glob

from setuptools import setup

package_name = 'navigation_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JingCheng',
    maintainer_email='1991245949@qq.com',
    description='Nav2 launch and configuration files for Air-UE-project.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dwb_goal_path_node = navigation_bringup.dwb_goal_path_node:main',
            'lidar_self_filter_node = navigation_bringup.lidar_self_filter_node:main',
            'odom_qos_relay_node = navigation_bringup.odom_qos_relay_node:main',
        ],
    },
)
