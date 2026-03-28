#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from my_command.srv import SequenceCmd 
import math
import time

class RotationControlNode(Node):
    def __init__(self):
        super().__init__('rotation_control_node')
        self.group = ReentrantCallbackGroup()

        # --- [ Parameters ] ---
        self.declare_parameter('kp_yaw', 0.45)
        self.declare_parameter('max_ang_vel', 0.5)
        self.declare_parameter('min_ang_vel', 0.15)
        self.declare_parameter('tolerance', 1.2)

        # Variables
        self.current_yaw_deg = 0.0
        self.target_yaw_deg = 0.0
        self.is_rotating = False
        self.rotate_dir = 0 # 1: บังคับซ้าย (ทวนเข็ม), -1: บังคับขวา (ตามเข็ม)

        # --- [ QoS & Sub/Pub/Srv ] ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom/filtered', self.odom_callback, qos_profile, callback_group=self.group)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_rotation', 10)
        self.srv = self.create_service(SequenceCmd, 'rotate_service', self.handle_rotate_service, callback_group=self.group)
        self.create_timer(0.05, self.control_loop, callback_group=self.group)

        self.get_logger().info("🚀 Rotation Control (Force Direction Mode) Started!")

    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def odom_callback(self, msg):
        self.current_yaw_deg = self.quaternion_to_euler(msg.pose.pose.orientation)
        if not self.is_rotating:
            self.get_logger().info(f"📍 Current Yaw: {self.current_yaw_deg:.2f}°", throttle_duration_sec=1.0)

    def handle_rotate_service(self, request, response):
        cmd = request.state.lower().strip()
        angle_to_turn = 0.0
        
        # --- ล็อคทิศทางตามชื่อคำสั่ง ---
        if 'left' in cmd:
            self.rotate_dir = 1  # บังคับหมุนทวนเข็มนาฬิกา (Positive)
        elif 'right' in cmd:
            self.rotate_dir = -1 # บังคับหมุนตามเข็มนาฬิกา (Negative)
        else:
            self.rotate_dir = 0  # กรณีอื่นๆ ให้หาทางสั้นที่สุด (Default)

        # กำหนดมุมที่ต้องหมุน
        if '90' in cmd: angle_to_turn = 90.0 * self.rotate_dir
        elif '180' in cmd: angle_to_turn = 180.0 * self.rotate_dir
        else:
            self.get_logger().error(f"❌ Unknown Command: {cmd}")
            response.status = "FAILED"
            return response

        # คำนวณเป้าหมาย
        start_angle = self.current_yaw_deg
        self.target_yaw_deg = start_angle + angle_to_turn
        
        # Normalize Target
        while self.target_yaw_deg > 180: self.target_yaw_deg -= 360
        while self.target_yaw_deg < -180: self.target_yaw_deg += 360

        self.is_rotating = True
        self.get_logger().info(f"🔄 ROTATING {cmd.upper()} | From: {start_angle:.2f}° | Target: {self.target_yaw_deg:.2f}°")

        while self.is_rotating and rclpy.ok():
            time.sleep(0.1)

        response.status = "SUCCESS"
        response.angle = self.current_yaw_deg
        return response

    def control_loop(self):
        if not self.is_rotating:
            return

        # 1. คำนวณ Error เบื้องต้น
        error = self.target_yaw_deg - self.current_yaw_deg
        
        # 2. ปรับจูน Error ตามทิศทางที่บังคับ (Force Direction)
        # ถ้าบังคับขวา (Right) แต่ Error ดันเป็นบวก ต้องลบ 360 เพื่อให้มันหมุนอ้อมไปทางขวาแทน
        if self.rotate_dir == -1 and error > 0:
            error -= 360
        # ถ้าบังคับซ้าย (Left) แต่ Error ดันเป็นลบ ต้องบวก 360 เพื่อให้มันหมุนอ้อมไปทางซ้ายแทน
        elif self.rotate_dir == 1 and error < 0:
            error += 360
        # ถ้าเป็นกรณีทั่วไป (Shortest path)
        elif self.rotate_dir == 0:
            while error > 180: error -= 360
            while error < -180: error += 360

        self.get_logger().info(f"📈 Progress: Current {self.current_yaw_deg:>7.2f}° | Error {error:>7.2f}°", throttle_duration_sec=0.2)

        msg = Twist()
        tolerance = self.get_parameter('tolerance').value

        if abs(error) < tolerance:
            self.cmd_pub.publish(Twist()) 
            self.is_rotating = False
            self.get_logger().info(f"🎯 REACHED TARGET! Final Angle: {self.current_yaw_deg:.2f}°")
            return

        kp = self.get_parameter('kp_yaw').value
        max_v = self.get_parameter('max_ang_vel').value
        min_v = self.get_parameter('min_ang_vel').value

        speed = error * kp
        
        if abs(speed) > max_v: speed = max_v if speed > 0 else -max_v
        if abs(speed) < min_v: speed = min_v if speed > 0 else -min_v

        msg.angular.z = speed
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RotationControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()