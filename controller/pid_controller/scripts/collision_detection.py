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
            points_gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            cloud_points = np.fromiter(points_gen, dtype=np.float32).reshape(-1, 3)
        except Exception as e:
            self.get_logger().error(f"Error reading PointCloud data: {e}")
            return

        if cloud_points.shape[0] == 0:
            return

        # Tính khoảng cách Euclidean
        dists = np.linalg.norm(cloud_points, axis=1)

        # Lọc điểm trong khoảng [min_dist, max_dist]
        mask = (dists > self.min_dist) & (dists < self.max_dist)
        valid_points = dists[mask]
        count = valid_points.shape[0]

        if count >= self.min_points:
            nearest = np.min(valid_points)
            self.get_logger().warn(
                f"⚠️ Collision detected! {count} points within [{self.min_dist}, {self.max_dist}] m. Nearest = {nearest:.2f} m"
            )
        else:
            self.get_logger().info(
                f"No collision: only {count} points within [{self.min_dist}, {self.max_dist}] m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarCollisionChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
