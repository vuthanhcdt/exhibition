#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class LidarCollisionChecker(Node):
    def __init__(self):
        super().__init__('lidar_collision_checker')

        # Subscription to PointCloud2 topic
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.pointcloud_callback, 10)

        # Parameters
        self.min_dist = 0.3    # m
        self.max_dist = 2.0    # m
        self.min_points = 50   # số điểm tối thiểu để coi là va chạm

    def pointcloud_callback(self, msg: PointCloud2):
        try:
            # Convert PointCloud2 -> numpy (N,3)
            points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            cloud_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)

            # Tính khoảng cách Euclidean từ gốc (0,0,0)
            dists = np.linalg.norm(cloud_points, axis=1)

            # Lọc điểm có khoảng cách trong [0.3, 0.6]
            mask = (dists > 0.3) & (dists < 0.6)
            num_points = np.count_nonzero(mask)

            # In/log số điểm tìm được
            self.get_logger().info(f"Số điểm trong khoảng 0.3 - 0.6 m: {num_points}")

        except Exception as e:
            self.get_logger().error(f"Error reading PointCloud data: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = LidarCollisionChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
