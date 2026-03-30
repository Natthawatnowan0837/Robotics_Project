#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from pynput import keyboard
import math

class KeyboardRotationNode(Node):
    def __init__(self):
        super().__init__('keyboard_rotation_node')
        self.group = ReentrantCallbackGroup()

        # --- [ Parameters ] ---
        # ปรับค่าเหล่านี้เพื่อความนุ่มนวลในการหมุน
        self.declare_parameter('kp_yaw', 0.6)         # ความเร็วแปรผันตาม Error
        self.declare_parameter('max_ang_vel', 0.7)    # ความเร็วสูงสุด
        self.declare_parameter('min_ang_vel', 0.15)   # ความเร็วต่ำสุดกันมอเตอร์ค้าง
        self.declare_parameter('tolerance', 1.5)      # จุดยอมรับการหยุด (องศา)

        # Variables
        self.current_yaw_deg = 0.0
        self.start_yaw_deg = 0.0   
        self.target_relative_deg = 0.0 
        self.is_rotating = False

        # --- [ QoS & Sub/Pub ] ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # รับค่าตำแหน่งจาก Odom
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/filtered', self.odom_callback, qos_profile, callback_group=self.group)
        
        # ส่งคำสั่งความเร็ว (เปลี่ยนชื่อ topic ตามหุ่นยนต์ของคุณ เช่น 'cmd_vel')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Keyboard Listener (ทำงานใน Thread แยก)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # Timer สำหรับคำนวณการเคลื่อนที่ (20Hz)
        self.create_timer(0.05, self.control_loop, callback_group=self.group)
        
        self.get_logger().info("🚀 Node Ready!")
        self.get_logger().info("Press 'q' to Turn Left 90° | 'e' to Turn Right 90° | 'ESC' to Quit")

    def quaternion_to_euler(self, q):
        """ แปลงค่า Quaternion เป็นองศา (Yaw) """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def odom_callback(self, msg):
        self.current_yaw_deg = self.quaternion_to_euler(msg.pose.pose.orientation)

    def on_press(self, key):
        """ จัดการเมื่อมีการกดปุ่ม """
        try:
            if hasattr(key, 'char'):
                if key.char == 'q':
                    self.initiate_rotation(90.0)  # หมุนซ้าย
                elif key.char == 'e':
                    self.initiate_rotation(-90.0) # หมุนขวา
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        
        if key == keyboard.Key.esc:
            self.get_logger().info("Shutting down...")
            rclpy.shutdown()

    def initiate_rotation(self, relative_angle):
        """ ตั้งค่าเริ่มต้นก่อนเริ่มหมุน """
        if self.is_rotating:
            self.get_logger().warn("⚠️ Robot is already rotating!")
            return
            
        self.start_yaw_deg = self.current_yaw_deg
        self.target_relative_deg = relative_angle
        self.is_rotating = True
        self.get_logger().info(f"🔄 Rotating {relative_angle}° from {self.start_yaw_deg:.2f}°")

    def control_loop(self):
        """ ลูปควบคุม PID อย่างง่ายเพื่อหมุนหุ่นยนต์ """
        if not self.is_rotating:
            return

        # 1. คำนวณหา "ระยะที่หมุนไปแล้ว" เทียบกับจุดเริ่มต้น
        relative_current_yaw = self.current_yaw_deg - self.start_yaw_deg
        
        # จัดการรอยต่อองศา (Wrapping) เช่น จาก 179 ไป -179
        if relative_current_yaw > 180: relative_current_yaw -= 360
        elif relative_current_yaw < -180: relative_current_yaw += 360

        # 2. คำนวณ Error (เป้าหมาย - ระยะที่มาถึงแล้ว)
        error = self.target_relative_deg - relative_current_yaw
        
        # จัดการ Wrapping ของ Error อีกครั้งเพื่อหาทิศที่ใกล้ที่สุด
        if error > 180: error -= 360
        elif error < -180: error += 360
        
        # 3. ตรวจสอบว่าถึงระยะที่ยอมรับได้หรือยัง
        if abs(error) < self.get_parameter('tolerance').value:
            self.cmd_pub.publish(Twist()) # ส่งค่า 0 เพื่อหยุด
            self.is_rotating = False
            self.get_logger().info("✅ Target Reached")
            return

        # 4. คำนวณความเร็ว (P-Control)
        kp = self.get_parameter('kp_yaw').value
        max_v = self.get_parameter('max_ang_vel').value
        min_v = self.get_parameter('min_ang_vel').value

        speed = error * kp
        
        # จำกัดความเร็ว (Saturation)
        if abs(speed) > max_v: speed = max_v if speed > 0 else -max_v
        if abs(speed) < min_v: speed = min_v if speed > 0 else -min_v

        msg = Twist()
        msg.angular.z = speed
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardRotationNode()
    
    # ใช้ MultiThreadedExecutor เพื่อให้ Subscriber และ Timer ทำงานขนานกันได้
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()