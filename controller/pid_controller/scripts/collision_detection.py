#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np



class LidarCollisionChecker(Node):
    def __init__(self):
        super().__init__('lidar_collision_checker')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.lidar_callback,
            10)
        
        # Ngưỡng khoảng cách
        self.min_dist = 0.3   # m
        self.max_dist = 0.5  # m
        self.min_points = 10  # số điểm tối thiểu để coi là chạm

        self.get_logger().info("Lidar Collision Checker Node Started")

    def lidar_callback(self, msg: PointCloud2):
        # Convert sang numpy (N,3)
        pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        points = ros_numpy.point_cloud2.get_xyz_points(pc_array, remove_nans=True)

        # Tính khoảng cách Euclidean
        dists = np.linalg.norm(points, axis=1)

        # Lọc trong khoảng min_dist < d < max_dist
        mask = (dists > self.min_dist) & (dists < self.max_dist)
        valid_points = points[mask]
        count = valid_points.shape[0]

        if count >= self.min_points:
            nearest = np.min(dists[mask])
            self.get_logger().warn(
                f"⚠️ Collision likely! {count} points in range [{self.min_dist}, {self.max_dist}] m, nearest {nearest:.2f} m"
            )
        else:
            self.get_logger().info(
                f"No collision: only {count} points in range [{self.min_dist}, {self.max_dist}] m"
            )

def main(args=None):
    rclpy.init(args=args)
    node = LidarCollisionChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()