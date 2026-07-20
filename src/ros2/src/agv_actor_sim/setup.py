from setuptools import find_packages, setup

package_name = 'agv_actor_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件, 之后可以 ros2 launch agv_actor_sim agv_actor_sim.launch.py
        ('share/' + package_name + '/launch', [
            'launch/agv_actor_sim.launch.py',
            'launch/agv_circle.launch.py',
            'launch/agv_keyboard.launch.py',
            'launch/agv_waypoint_cruise.launch.py',
        ]),
        # 巡航航点 YAML, 让 cruise 节点能用 share 路径找到
        ('share/' + package_name, ['AGV_Traj.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JingCheng',
    maintainer_email='1991245949@qq.com',
    description='Pseudo AGV (UE actor) ROS2 control + IMU/Odom publisher.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 持续运行的两个常驻节点
            'agv_actor_node = agv_actor_sim.agv_actor_node:main',
            'agv_imu_odom_node = agv_actor_sim.agv_imu_odom_node:main',
            'agv_circle_node = agv_actor_sim.agv_circle_node:main',
            'agv_figure_eight_node = agv_actor_sim.agv_figure_eight_node:main',
            'agv_keyboard_teleop_node = agv_actor_sim.agv_keyboard_teleop_node:main',
            'agv_waypoint_cruise_node = agv_actor_sim.agv_waypoint_cruise_node:main',
            # 一次性闭环测试节点
            'agv_test_drive_node = agv_actor_sim.agv_test_drive_node:main',
        ],
    },
)
