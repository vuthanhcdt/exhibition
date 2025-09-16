#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        # self.pose_sub = self.create_subscription(
        #     Pose,
        #     '/human_local_pose',
        #     self.pose_callback,
        #     10
        # )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.current_pose = None  # Lưu pose mới nhất
        self.distance_robot_human = 0.0  
        # Timer xuất lệnh điều khiển mỗi 0.001 giây (1000Hz)
        self.timer = self.create_timer(0.001, self.timer_callback)
         # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def PID(self, error, Kp):
        output = Kp * error
        return output
    
    def get_transform(self, target_frame, source_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            self.get_logger().info(
                f"Transform from {source_frame} to {target_frame}: "
                f"translation=({t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z})"
            )
            return t
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform from {source_frame} to {target_frame}: {e}")
            return None

    def pose_callback(self, msg):
        # Đặt mục tiêu đi cạnh người (bên phải 0.5m)
        target_x = 0.0  # Không đi trước/sau người
        target_y = 0.0  # Bên phải người 0.5m (đổi dấu nếu muốn bên trái)
        error_x = msg.position.x - target_x
        error_y = msg.position.y - target_y
        self.distance_robot_human = ((error_x)**2 + (error_y)**2)**0.5
        self.linear_x = self.PID(self.distance_robot_human-1.0, 1.0) # Giữ khoảng cách 0.5m
        self.linear_x = max(min(self.linear_x, 2.0), -1.5)  # Giới hạn tốc độ
        self.relative_angle = math.atan2(error_y, error_x)
        self.angular_z = self.PID(self.relative_angle, 2.0)
        if self.linear_x < 0.05:
            self.angular_z = 0.0
        self.angular_z = max(min(self.angular_z, 1.0), -1.0)  # Giới hạn tốc độ
        # self.get_logger().info(f"Received pose: x={msg.position.x}, y={msg.position.y}, distance={self.distance_robot_human}, angle={self.relative_angle}")
        self.current_pose = msg

    def timer_callback(self):
        right_tf = self.get_transform('base_link', 'right_following')
        left_tf = self.get_transform('base_link', 'left_following')
        self.cmd = Twist()

        # Tính khoảng cách nếu có transform
        left_dist = None
        right_dist = None
        if left_tf is not None:
            left_x = left_tf.transform.translation.x
            left_y = left_tf.transform.translation.y
            left_dist = math.hypot(left_x, left_y)
        if right_tf is not None:
            right_x = right_tf.transform.translation.x
            right_y = right_tf.transform.translation.y
            right_dist = math.hypot(right_x, right_y)

        # Chọn transform gần nhất
        chosen_tf = None
        if left_dist is not None and right_dist is not None:
            if left_dist <= right_dist:
                chosen_tf = left_tf
            else:
                chosen_tf = right_tf
        elif left_dist is not None:
            chosen_tf = left_tf
        elif right_dist is not None:
            chosen_tf = right_tf

        # Điều khiển robot theo transform gần nhất
        if chosen_tf is not None:
            error_x = chosen_tf.transform.translation.x
            error_y = chosen_tf.transform.translation.y
            distance = math.hypot(error_x, error_y)
            linear_x = self.PID(distance - 0.3, 1.0)  # Giữ khoảng cách 0.3m (bạn có thể đổi)
            linear_x = max(min(linear_x, 2.0), -1.5)
            relative_angle = math.atan2(error_y, error_x)
            angular_z = self.PID(relative_angle, 2.0)
            if abs(linear_x) < 0.05:
                angular_z = 0.0
            angular_z = max(min(angular_z, 1.0), -1.0)
            self.cmd.linear.x = linear_x
            self.cmd.angular.z = angular_z
            self.cmd_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()