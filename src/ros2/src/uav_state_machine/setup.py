import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_state_machine'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JingCheng',
    maintainer_email='1991245949@qq.com',
    description='Simple ROS2 state machine node for UAV behavior switching.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uav_state_machine_node = uav_state_machine.uav_state_machine_node:main',
        ],
    },
)
