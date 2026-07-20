from setuptools import find_packages, setup

package_name = 'coordination'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/coordination_demo.launch.py',
            'launch/coordination_rtl_demo.launch.py',
            'launch/yoloe_test_demo.launch.py',
            'launch/agv_chase_demo.launch.py',
            # 新增: coni-mpc 降落 demo (基于 chase demo 之上叠加 MPC 接管降落).
            'launch/mpc_landing_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JingCheng',
    maintainer_email='1991245949@qq.com',
    description='Simple UAV/UGV coordination nodes for the Air-UE-project demos.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ugv_then_uav_node = coordination.ugv_then_uav_node:main',
            'uav_recovery_state_node = coordination.uav_recovery_state_node:main',
            'uav_follow_agv_node = coordination.uav_follow_agv_node:main',
            # 新增节点: 带返航 (RTL) 与 YOLOE 进近的协调节点, 以及 YOLOE 检测发布节点.
            'uav_with_rtl_node = coordination.uav_with_rtl_node:main',
            'yoloe_detector_node = coordination.yoloe_detector_node:main',
            # YOLOE 识别测试节点: 解绑 -> 起飞 10m -> 后退 10m -> 悬停.
            'uav_yoloe_test_node = coordination.uav_yoloe_test_node:main',
            # 追踪 AGV: 等 AGV 跑 5s -> 起飞 -> 巡航追 AGV GPS -> 距离 15m 处悬停.
            'uav_chase_agv_node = coordination.uav_chase_agv_node:main',
            # coni-mpc 降落: AttitudeTarget -> AirSim 低层 API 桥接 + 降落协调.
            'mpc_attitude_bridge_node = coordination.mpc_attitude_bridge_node:main',
            'mpc_land_coordinator_node = coordination.mpc_land_coordinator_node:main',
        ],
    },
)
