#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('keyboard_controller_node')
        
        # --- [ Configuration ] ---
        # ปรับค่าความเร็วตรงนี้ให้เหมาะกับหุ่นยนต์ของคุณ
        self.declare_parameter('linear_speed', 0.3) 
        self.declare_parameter('angular_speed', 0.3)
        
        self.linear_max = self.get_parameter('linear_speed').value
        self.angular_max = self.get_parameter('angular_speed').value

        self.get_logger().info(f"--- Keyboard Controller Started ---")
        self.get_logger().info(f"Use WASD to move. Press 'ESC' to stop node.")
        self.get_logger().info(f"Current Config: Linear {self.linear_max}, Angular {self.angular_max}")

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # เก็บสถานะการกดปุ่ม
        self.pressed_keys = set()
        
        # เริ่มต้นฟังเสียงจากคีย์บอร์ด (Non-blocking)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        # Timer สำหรับส่งคำสั่ง cmd_vel อย่างต่อเนื่อง (20Hz)
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)

    def on_press(self, key):
        try:
            if hasattr(key, 'char'):
                self.pressed_keys.add(key.char.lower())
        except Exception:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char'):
                char = key.char.lower()
                if char in self.pressed_keys:
                    self.pressed_keys.remove(char)
            
            # กด ESC เพื่อปิดโปรแกรม
            if key == keyboard.Key.esc:
                self.get_logger().info("Exiting...")
                rclpy.shutdown()
        except Exception:
            pass

    def publish_cmd_vel(self):
        twist = Twist()
        
        # คำนวณความเร็วจากปุ่มที่กด
        linear_val = 0.0
        angular_val = 0.0

        if 'w' in self.pressed_keys:
            linear_val += self.linear_max
        if 's' in self.pressed_keys:
            linear_val -= self.linear_max
        if 'a' in self.pressed_keys:
            angular_val += self.angular_max
        if 'd' in self.pressed_keys:
            angular_val -= self.angular_max

        twist.linear.x = float(linear_val)
        twist.angular.z = float(angular_val)
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # ส่งค่า 0 เพื่อหยุดหุ่นยนต์ก่อนปิด Node
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()