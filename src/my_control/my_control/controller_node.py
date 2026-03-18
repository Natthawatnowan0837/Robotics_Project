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
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('arm_speed', 0.5) # เพิ่มความเร็วสำหรับแขนกล
        
        self.linear_max = self.get_parameter('linear_speed').value
        self.angular_max = self.get_parameter('angular_speed').value
        self.arm_max = self.get_parameter('arm_speed').value

        self.get_logger().info(f"--- Keyboard Controller Started ---")
        self.get_logger().info(f"WASD: Move Robot | QE: Move Arm | ESC: Stop")
        self.get_logger().info(f"Config: Linear {self.linear_max}, Angular {self.angular_max}, Arm {self.arm_max}")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_vel_pub = self.create_publisher(Twist, 'arm_vel', 10) # เพิ่ม Topic สำหรับแขน
        
        self.pressed_keys = set()
        
        # คีย์บอร์ด Listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        # Timer (20Hz)
        self.timer = self.create_timer(0.05, self.publish_commands)

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
            
            if key == keyboard.Key.esc:
                self.get_logger().info("Exiting...")
                rclpy.shutdown()
        except Exception:
            pass

    def publish_commands(self):
        # --- จัดการหุ่นยนต์ (Base) ---
        twist = Twist()
        if 'w' in self.pressed_keys: twist.linear.x += self.linear_max
        if 's' in self.pressed_keys: twist.linear.x -= self.linear_max
        if 'a' in self.pressed_keys: twist.angular.z += self.angular_max
        if 'd' in self.pressed_keys: twist.angular.z -= self.angular_max
        self.cmd_vel_pub.publish(twist)

        # --- จัดการแขนกล (Arm) ---
        arm_twist = Twist()
        if 'q' in self.pressed_keys:
            arm_twist.linear.x = float(self.arm_max)
        if 'e' in self.pressed_keys:
            arm_twist.linear.x = float(-self.arm_max)
        
        self.arm_vel_pub.publish(arm_twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # หยุดทุกอย่างก่อนปิด
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.arm_vel_pub.publish(stop_msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()