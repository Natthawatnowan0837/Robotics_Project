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
        self.declare_parameter('kp_yaw', 0.6)
        self.declare_parameter('max_ang_vel', 0.6)
        self.declare_parameter('min_ang_vel', 0.15)
        self.declare_parameter('linear_speed', 0.25) 
        self.declare_parameter('tolerance', 1.5) # ปรับให้กว้างขึ้นนิดนึงเพื่อความเร็ว

        # Variables
        self.current_yaw_deg = 0.0
        self.start_yaw_deg = 0.0   # ใช้เก็บค่ามุม "ก่อนเริ่มหมุน" เพื่อใช้เป็นจุด Reset 0
        self.target_relative_deg = 0.0 
        self.is_rotating = False
        self.is_moving_fwd = False 

        # --- [ QoS & Sub/Pub/Srv ] ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/filtered', self.odom_callback, qos_profile, callback_group=self.group)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_rotation', 10)
        self.srv = self.create_service(
            SequenceCmd, 'rotate_service', self.handle_rotate_service, callback_group=self.group)
        
        self.create_timer(0.05, self.control_loop, callback_group=self.group)
        self.get_logger().info("🚀 Relative Rotation Mode: Resetting to 0 before every turn.")

    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def odom_callback(self, msg):
        self.current_yaw_deg = self.quaternion_to_euler(msg.pose.pose.orientation)

    def handle_rotate_service(self, request, response):
        cmd = request.state.lower().strip()
        
        if cmd == 'stop':
            self.is_rotating = False
            self.is_moving_fwd = False 
            self.stop_robot()
            response.status = "STOPPED"
            return response

        if cmd == 'fwd':
            self.get_logger().info("🏃 FWD 0.5s")
            self.is_moving_fwd = True
            msg = Twist(); msg.linear.x = self.get_parameter('linear_speed').value
            self.cmd_pub.publish(msg)
            time.sleep(0.5)
            self.stop_robot()
            self.is_moving_fwd = False
            response.status = "SUCCESS"
            return response

        # --- ส่วนการหมุนแบบ Reset 0 ---
        # 1. บันทึกค่ามุมปัจจุบันไว้เป็นจุดเริ่ม (เสมือนเป็น 0 องศา)
        self.start_yaw_deg = self.current_yaw_deg
        
        # 2. กำหนดมุมที่จะหมุนจากจุดปัจจุบัน
        if 'left 90' in cmd: self.target_relative_deg = 90.0
        elif 'right 90' in cmd: self.target_relative_deg = -90.0
        elif 'left 180' in cmd: self.target_relative_deg = 180.0
        elif 'right 180' in cmd: self.target_relative_deg = -180.0
        else:
            response.status = "FAILED"
            return response

        self.is_rotating = True
        self.get_logger().info(f"🔄 Resetting reference to 0. Rotating {cmd.upper()}...")

        while self.is_rotating and rclpy.ok():
            time.sleep(0.05)

        response.status = "SUCCESS"
        return response

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if not self.is_rotating or self.is_moving_fwd:
            return

        # คำนวณหา "มุมที่หมุนมาแล้วจริงๆ" นับจากจุดเริ่ม (Relative Yaw)
        relative_current_yaw = self.current_yaw_deg - self.start_yaw_deg
        
        #จัดการ Wrap Angle ให้การลบกันยังอยู่ในช่วง -180 ถึง 180
        if relative_current_yaw > 180: relative_current_yaw -= 360
        elif relative_current_yaw < -180: relative_current_yaw += 360

        error = self.target_relative_deg - relative_current_yaw
        
        # จัดการ Shortest Path
        if error > 180: error -= 360
        elif error < -180: error += 360
        
        if abs(error) < self.get_parameter('tolerance').value:
            self.stop_robot()
            self.is_rotating = False
            return

        speed = error * self.get_parameter('kp_yaw').value
        max_v = self.get_parameter('max_ang_vel').value
        min_v = self.get_parameter('min_ang_vel').value
        
        if abs(speed) > max_v: speed = max_v if speed > 0 else -max_v
        if abs(speed) < min_v: speed = min_v if speed > 0 else -min_v

        msg = Twist(); msg.angular.z = speed
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = RotationControlNode()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()