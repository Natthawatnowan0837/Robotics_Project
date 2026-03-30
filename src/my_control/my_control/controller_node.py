#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray  # แก้ไข import ให้ถูกต้อง
from pynput import keyboard

class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('keyboard_controller_node')
        
        # --- [ Configuration ] ---
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('arm_speed', 0.5)
        
        self.linear_max = self.get_parameter('linear_speed').value
        self.angular_max = self.get_parameter('angular_speed').value
        self.arm_max = self.get_parameter('arm_speed').value

        # State management
        self.current_body_angle = 0.0
        self.target_angle = None
        self.is_auto_turning = False
        self.pressed_keys = set()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_vel_pub = self.create_publisher(Twist, 'arm_vel', 10)
        
        # Subscriber: รับค่าจากเซนเซอร์ตัวที่ 2 (Index 1)
        self.create_subscription(
            Float32MultiArray, 
            '/sensors', 
            self.angle_callback, 
            10)

        # Keyboard Listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Timer (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("--- Keyboard Controller Node Started ---")
        self.get_logger().info("WASD: Move | QE: Arm | J/L: Auto 90° Turn | ESC: Quit")

    def angle_callback(self, msg):
        """ ฟังก์ชันรับค่าจาก sensor topic """
        if len(msg.data) >= 2:
            # เก็บค่าตัวที่ 2 (index 1) ไว้ใช้คำนวณการเลี้ยว
            self.current_body_angle = msg.data[2]

    def on_press(self, key):
        try:
            if hasattr(key, 'char'):
                key_char = key.char.lower()
                self.pressed_keys.add(key_char)
                
                # สั่งเลี้ยวอัตโนมัติเมื่อกด J หรือ L
                if key_char == 'l' and not self.is_auto_turning:
                    self.start_auto_turn(-180.0) # เลี้ยวขวา
                elif key_char == 'j' and not self.is_auto_turning:
                    self.start_auto_turn(180.0)  # เลี้ยวซ้าย
        except:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char'):
                char = key.char.lower()
                if char in self.pressed_keys:
                    self.pressed_keys.remove(char)
            if key == keyboard.Key.esc:
                self.get_logger().info("Exiting on ESC...")
                rclpy.shutdown()
        except:
            pass

    def start_auto_turn(self, angle_offset):
        """ คำนวณเป้าหมายและเริ่มโหมดเลี้ยวอัตโนมัติ """
        self.target_angle = self.current_body_angle + angle_offset
        self.is_auto_turning = True
        self.get_logger().info(f"Auto Turn Start: Target {self.target_angle:.2f}")

    def control_loop(self):
        """ Loop หลักในการคำนวณและ Publish คำสั่ง """
        twist = Twist()
        arm_twist = Twist()

        # 1. ระบบเลี้ยวอัตโนมัติ (Autonomous Turn)
        if self.is_auto_turning:
            error = self.target_angle - self.current_body_angle
            
            # เมื่อใกล้ถึงเป้าหมาย (Tolerance 2 องศา)
            if abs(error) < 2.0:
                self.is_auto_turning = False
                self.target_angle = None
                self.get_logger().info("Target Reached.")
            else:
                # ส่งค่าความเร็วเชิงมุมเพื่อหมุน
                twist.angular.z = self.angular_max if error > 0 else -self.angular_max
        
        # 2. ระบบควบคุมด้วยมือ (Manual Control) - ทำงานเมื่อไม่ได้เลี้ยวอัตโนมัติ
        else:
            if 'w' in self.pressed_keys: twist.linear.x = self.linear_max
            if 's' in self.pressed_keys: twist.linear.x = -self.linear_max
            if 'a' in self.pressed_keys: twist.angular.z = self.angular_max
            if 'd' in self.pressed_keys: twist.angular.z = -self.angular_max

        # 3. ระบบควบคุมแขนกล
        if 'q' in self.pressed_keys: arm_twist.linear.x = self.arm_max
        if 'e' in self.pressed_keys: arm_twist.linear.x = -self.arm_max

        self.cmd_vel_pub.publish(twist)
        self.arm_vel_pub.publish(arm_twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # สั่งหยุดหุ่นยนต์ก่อนปิดโปรแกรม
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()