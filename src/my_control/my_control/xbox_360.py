#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        
        self.move_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.multiplier = 1.0  
        
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller found!")
            return
            
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(f"Xbox Ready (Twist Analog Mode) | Max Speed Multiplier: {self.multiplier}")

        self.timer = self.create_timer(0.05, self.controller_loop)

    def controller_loop(self):
        pygame.event.pump()
        msg = Twist()

        # --- ส่วนของ Multiplier Logic (เพิ่มกลับเข้าไปให้) ---
        if self.joy.get_button(0): # A: ลดความเร็ว
            self.multiplier = max(0.1, self.multiplier - 0.1)
            self.get_logger().info(f"Speed Multiplier: {self.multiplier:.1f}")
        elif self.joy.get_button(3): # Y หรือ X (ตาม Index): เพิ่มความเร็ว
            self.multiplier = min(2.0, self.multiplier + 0.1)
            self.get_logger().info(f"Speed Multiplier: {self.multiplier:.1f}")

        # 1. อ่านค่าดิบจาก Analog
        raw_linear = -self.joy.get_axis(1) 
        raw_angular = -self.joy.get_axis(0)

        # 2. ตั้งค่า Deadzone 
        deadzone = 0.15

        # ตรวจสอบแกน Linear
        if abs(raw_linear) < deadzone:
            msg.linear.x = 0.0
        else:
            msg.linear.x = raw_linear * 0.5 * self.multiplier

        # ตรวจสอบแกน Angular
        if abs(raw_angular) < deadzone:
            msg.angular.z = 0.0
        else:
            msg.angular.z = raw_angular * 1.5 * self.multiplier

        # 3. ส่งข้อมูล (Publish)
        self.move_pub.publish(msg)

        # 4. แสดงผลเพื่อเช็คใน Terminal
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.get_logger().info(f"Moving: L={msg.linear.x:.2f}, A={msg.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()