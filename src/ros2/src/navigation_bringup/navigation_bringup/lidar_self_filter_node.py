"""过滤掉打在无人机机身上的激光点云。

激光雷达装在无人机上，近场点云常打到机身/机臂/桨叶，会被 DWB 当成虚假障碍物。
本节点用一个与雷达 z 轴对齐的圆柱体作为排除区域（xy 半径 + abs(z) 半高），
只剔除机身自身的反射点，而保留圆柱体外的墙面和地面点。
"""
from __future__ import annotations

import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class LidarSelfFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_self_filter_node')

        self.declare_parameter('input_topic', '/uav/airsim_node/UAV_1/lidar/points/UAV_1_Lidar1')
        self.declare_parameter('output_topic', '/uav/lidar/points_filtered')
        # 机身圆柱体尺寸（米），以雷达坐标系原点为中心，根据实测机身反射点调校。
        self.declare_parameter('self_radius_xy', 0.60)
        self.declare_parameter('self_half_height', 0.25)
        self.self_radius_xy = float(self.get_parameter('self_radius_xy').value)
        self.self_half_height = float(self.get_parameter('self_half_height').value)
        self._radius_sq = self.self_radius_xy * self.self_radius_xy

        self.pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_topic').value),
            self._cb,
            10,
        )

    def _inside_self_envelope(self, x: float, y: float, z: float) -> bool:
        # 圆柱体排除：xy 在半径内且 abs(z) 在半高内才判为机身点（两个条件需同时满足）。
        if abs(z) > self.self_half_height:
            return False
        return (x * x + y * y) <= self._radius_sq

    def _cb(self, msg: PointCloud2) -> None:
        # 仅保留机身范围外的点，并重新发布为紧凑的 XYZ32 点云。
        filtered = []
        for x, y, z in pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True):
            if not self._inside_self_envelope(x, y, z):
                filtered.append((x, y, z))

        out = pc2.create_cloud_xyz32(msg.header, filtered)
        self.pub.publish(out)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = LidarSelfFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
