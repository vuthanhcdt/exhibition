#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Int32
from scout_msgs.msg import ScoutRCState
import math, time
from scipy.spatial.transform import Rotation as R
import numpy as np
from scout_msgs.msg import ScoutLightCmd

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

        self.human_state = 0  # Lưu trạng thái human_state (0 hoặc 1)
        self.human_state_sub = self.create_subscription(
            Int32,
            'human_state',
            self.human_state_callback,
            10
        )
        self.rc_sub = self.create_subscription(ScoutRCState,'/scout_rc_state', self.rc_callback,5)
        self.follow_mode = 0
        self.robot_mode = 2  # 0: stop, 1: following, 2: dancing
        self.light_pub = self.create_publisher(
            ScoutLightCmd,
            '/light_control',
            10
        )
        self.dance_mode = 0


    def human_state_callback(self, msg):
        self.human_state = msg.data
        # self.get_logger().info(f"Received human_state: {self.human_state}")
        
    
    @staticmethod
    def quaternion_to_yaw(quaternion):
        """Extract yaw angle from a quaternion."""
        return R.from_quat(quaternion).as_euler('xyz')[2]
    
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> np.ndarray:
        """Converts a yaw angle (in radians) to a quaternion."""
        return R.from_euler('xyz', [0.0, 0.0, yaw], degrees=False).as_quat()
    

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
            # self.get_logger().info(
            #     f"Transform from {source_frame} to {target_frame}: "
            #     f"translation=({t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z})"
            # )
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

    def rc_callback(self, msg):
      self.follow_mode = msg.swa

    def dancing1(self):
        # Số lần lắc đầu (mỗi lần sang trái/phải là 1 lần)
        NUM_SWINGS = 6
        # Góc lệch mỗi lần (radian)
        SWING_ANGLE = math.radians(30)  # 30 độ sang mỗi bên

        if not hasattr(self, 'dance_start_angle'):
            try:
                odom_tf = self.get_transform('odom', 'base_link')
                if odom_tf is not None:
                    q = odom_tf.transform.rotation
                    self.dance_start_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
                    self.dance_start_time = self.get_clock().now().nanoseconds / 1e9
                    self.dance_swing_idx = 0
                    self.dance_direction = 1  # 1: phải, -1: trái
                    self.dance_waiting = False
                else:
                    return
            except Exception as e:
                self.get_logger().warn(f"Could not get odom for dancing: {e}")
                return
            
        self.cmd = Twist()
        self.cmd.linear.x = 0.0

        try:
            odom_tf = self.get_transform('odom', 'base_link')
            if odom_tf is not None:
                q = odom_tf.transform.rotation
                current_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
            else:
                return
        except Exception as e:
            self.get_logger().warn(f"Could not get odom for dancing: {e}")
            return

        # Tạo target angle cho từng lần lắc
        if self.dance_swing_idx < NUM_SWINGS:
            if not self.dance_waiting:
                # Xác định góc mục tiêu cho lần lắc này
                offset = SWING_ANGLE * self.dance_direction
                self.dance_target_angle = self.dance_start_angle + offset
                # Đưa về [-pi, pi]
                self.dance_target_angle = (self.dance_target_angle + math.pi) % (2 * math.pi) - math.pi
                self.dance_waiting = True

            # Tính sai số góc
            angle_error = (self.dance_target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

            if abs(angle_error) > 0.03:
                self.blink_light()
                self.cmd.angular.z = max(min(3.0 * angle_error, 2.0), -2.0)
                self.cmd_pub.publish(self.cmd)
            else:
                # Đã đến vị trí, đổi hướng cho lần tiếp theo
                self.dance_direction *= -1
                self.dance_swing_idx += 1
                self.dance_waiting = False
        else:
            # Quay về góc ban đầu
            angle_error = (self.dance_start_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_error) > 0.03:
                self.cmd.angular.z = max(min(2.0 * angle_error, 0.7), -0.7)
                self.cmd_pub.publish(self.cmd)
            else:
                self.robot_mode = 0
                for attr in ['dance_start_angle', 'dance_start_time', 'dance_swing_idx', 'dance_direction', 'dance_waiting', 'dance_target_angle']:
                    if hasattr(self, attr):
                        delattr(self, attr)
    def dancing2(self):
        # Xoay liên tục trong 5 giây với tốc độ góc cố định, sau đó trở lại góc ban đầu
        ROTATE_DURATION = 10.0  # giây
        ROTATE_SPEED = 2.5     # rad/s

        if not hasattr(self, 'dancing2_start_time'):
            self.dancing2_start_time = self.get_clock().now().nanoseconds / 1e9
            # Lưu lại góc ban đầu
            odom_tf = self.get_transform('odom', 'base_link')
            if odom_tf is not None:
                q = odom_tf.transform.rotation
                self.dancing2_start_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
            else:
                self.dancing2_start_angle = 0.0  # fallback

        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.dancing2_start_time

        self.cmd = Twist()
        if elapsed < ROTATE_DURATION:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = ROTATE_SPEED
            self.cmd_pub.publish(self.cmd)
            self.blink_light()
        else:
            # Quay về góc ban đầu
            odom_tf = self.get_transform('odom', 'base_link')
            if odom_tf is not None:
                q = odom_tf.transform.rotation
                current_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
                angle_error = (self.dancing2_start_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
                if abs(angle_error) > 0.03:
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = max(min(2.0 * angle_error, 0.7), -0.7)
                    self.cmd_pub.publish(self.cmd)
                    return
            # Kết thúc chế độ dancing2
            self.robot_mode = 0
            for attr in ['dancing2_start_time', 'dancing2_start_angle']:
                if hasattr(self, attr):
                    delattr(self, attr)

    def light_off(self):
        light_cmd = ScoutLightCmd()
        light_cmd.cmd_ctrl_allowed = True
        light_cmd.front_mode = 0  # LIGHT_CONST_OFF
        light_cmd.front_custom_value = 0
        light_cmd.rear_mode = 0   # LIGHT_CONST_OFF
        light_cmd.rear_custom_value = 0
        self.light_pub.publish(light_cmd)

    def blink_light(self):
        light_cmd = ScoutLightCmd()
        light_cmd.cmd_ctrl_allowed = True
        light_cmd.front_mode = 2  # LIGHT_BLINK
        light_cmd.front_custom_value = 100  # Tần số nhấp nháy
        light_cmd.rear_mode = 2   # LIGHT_BLINK
        light_cmd.rear_custom_value = 100
        self.light_pub.publish(light_cmd)


    def following_mode(self):
        right_tf = self.get_transform('base_link', 'right_following')
        left_tf = self.get_transform('base_link', 'left_following')
        center_tf = self.get_transform('base_link', 'human_link')
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
        if center_tf is not None:
            center_x = center_tf.transform.translation.x
            center_y = center_tf.transform.translation.y
            center_dist = math.hypot(center_x, center_y)

        # Chọn transform gần nhất
        chosen_tf = None

        if self.follow_mode > 0:
            if left_dist is not None and right_dist is not None:
                if left_dist <= right_dist:
                    chosen_tf = left_tf
                else:
                    chosen_tf = right_tf
            elif left_dist is not None:
                chosen_tf = left_tf
            elif right_dist is not None:
                chosen_tf = right_tf
            offset_distane = 0.0
        else:
            chosen_tf = center_tf
            offset_distane = 1.0

        # Điều khiển robot theo transform gần nhất
        if chosen_tf is not None:
            error_x = chosen_tf.transform.translation.x
            error_y = chosen_tf.transform.translation.y
            distance = math.hypot(error_x, error_y)
            linear_x = self.PID(distance - offset_distane, 1.0)  # Giữ khoảng cách 1.0m (bạn có thể đổi)
            linear_x = max(min(linear_x, 2.0), -1.5)
            relative_angle = math.atan2(error_y, error_x)
            if error_x<-0.5:
                linear_x = 0.0
            # print(relative_angle)
            angular_z = self.PID(relative_angle, 2.0)
            angular_z = max(min(angular_z, 1.0), -1.0)
            if center_dist>0.5:
                if self.human_state == 1:
                    self.cmd.linear.x = linear_x
                    self.cmd.angular.z = angular_z
                    self.cmd_pub.publish(self.cmd)
                elif self.human_state == 0:
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd)
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.cmd_pub.publish(self.cmd)


    def timer_callback(self):
        # Chọn chế độ hoạt động: dancing, following, hay stop
        if self.robot_mode == 2:
            # Trường hợp nhảy (dancing)
            if self.dance_mode ==0:
                self.dancing1()
            else:
                self.dancing2()
        elif self.robot_mode == 1:
            self.light_off()
            self.following_mode()
        else:
            self.light_off()
            self.cmd = Twist()
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.cmd_pub.publish(self.cmd)
            return
        
        


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()