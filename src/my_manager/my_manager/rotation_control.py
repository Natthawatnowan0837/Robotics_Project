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
        self.rotate_dir = 0 

        # --- [ QoS & Sub/Pub/Srv ] ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom/filtered', self.odom_callback, qos_profile, callback_group=self.group)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_rotation', 10)
        
        # Service Server
        self.srv = self.create_service(SequenceCmd, 'rotate_service', self.handle_rotate_service, callback_group=self.group)
        
        # Control Loop Timer
        self.create_timer(0.05, self.control_loop, callback_group=self.group)

        self.get_logger().info("🚀 Rotation Control (With Emergency STOP) Started!")

    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def odom_callback(self, msg):
        self.current_yaw_deg = self.quaternion_to_euler(msg.pose.pose.orientation)

    def handle_rotate_service(self, request, response):
        cmd = request.state.lower().strip()
        
        # --- [ ส่วนที่เพิ่มเข้ามา: ตรวจสอบคำสั่ง STOP ] ---
        if cmd == 'stop':
            self.get_logger().warn("🛑 STOP command received! Halting rotation immediately.")
            self.is_rotating = False  # หยุด loop การทำงานใน service
            self.stop_robot()        # ส่งความเร็ว 0 ทันที
            response.status = "STOPPED"
            response.angle = self.current_yaw_deg
            return response
        # ----------------------------------------------

        angle_to_turn = 0.0
        
        if 'left' in cmd:
            self.rotate_dir = 1 
        elif 'right' in cmd:
            self.rotate_dir = -1
        else:
            self.rotate_dir = 0 

        if '90' in cmd: angle_to_turn = 90.0 * self.rotate_dir
        elif '180' in cmd: angle_to_turn = 180.0 * self.rotate_dir
        else:
            self.get_logger().error(f"❌ Unknown Command: {cmd}")
            response.status = "FAILED"
            return response

        start_angle = self.current_yaw_deg
        self.target_yaw_deg = start_angle + angle_to_turn
        
        # Normalize Target
        while self.target_yaw_deg > 180: self.target_yaw_deg -= 360
        while self.target_yaw_deg < -180: self.target_yaw_deg += 360

        self.is_rotating = True
        self.get_logger().info(f"🔄 ROTATING {cmd.upper()} | From: {start_angle:.2f}° | Target: {self.target_yaw_deg:.2f}°")

        # รอจนกว่าจะหมุนเสร็จ หรือถูกสั่ง STOP (is_rotating กลายเป็น False)
        while self.is_rotating and rclpy.ok():
            time.sleep(0.05)

        response.status = "SUCCESS" if not self.is_rotating else "INTERRUPTED"
        response.angle = self.current_yaw_deg
        return response

    def stop_robot(self):
        """ฟังก์ชันส่งความเร็วเป็น 0 ทันที"""
        msg = Twist()
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if not self.is_rotating:
            return

        error = self.target_yaw_deg - self.current_yaw_deg
        
        # ปรับจูน Error ตามทิศทางที่บังคับ
        if self.rotate_dir == -1 and error > 0:
            error -= 360
        elif self.rotate_dir == 1 and error < 0:
            error += 360
        elif self.rotate_dir == 0:
            while error > 180: error -= 360
            while error < -180: error += 360

        tolerance = self.get_parameter('tolerance').value

        # เช็คว่าถึงเป้าหมายหรือยัง
        if abs(error) < tolerance:
            self.stop_robot()
            self.is_rotating = False
            self.get_logger().info(f"🎯 REACHED TARGET! Final Angle: {self.current_yaw_deg:.2f}°")
            return

        # PID อย่างง่าย (P-Control)
        kp = self.get_parameter('kp_yaw').value
        max_v = self.get_parameter('max_ang_vel').value
        min_v = self.get_parameter('min_ang_vel').value

        speed = error * kp
        
        if abs(speed) > max_v: speed = max_v if speed > 0 else -max_v
        if abs(speed) < min_v: speed = min_v if speed > 0 else -min_v

        msg = Twist()
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