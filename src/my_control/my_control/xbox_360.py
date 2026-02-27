#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        
        # Publisher เดิมสำหรับตัวหุ่น
        self.move_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # --- เพิ่ม Publisher ใหม่สำหรับ Platform ---
        self.platform_pub = self.create_publisher(Twist, 'platform_cmd_vel', 10)
        
        self.multiplier = 1.0
        self.deadzone = 0.15
        
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("ไม่พบจอย Xbox!")
            return
            
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        
        self.get_logger().info("---------------------------------------------")
        self.get_logger().info("D-pad (ขึ้น/ลง) : ควบคุม Platform (platform_vel)")
        self.get_logger().info("Analog ซ้าย/ขวา : ควบคุมการเคลื่อนที่ (cmd_vel)")
        self.get_logger().info("---------------------------------------------")

        self.timer = self.create_timer(0.05, self.controller_loop)

    def controller_loop(self):
        # Event handling สำหรับปุ่มกด (Button)
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0: # A
                    self.multiplier = max(0.1, self.multiplier - 0.1)
                    self.get_logger().info(f"Speed: {self.multiplier:.1f}")
                elif event.button == 3: # Y
                    self.multiplier = min(2.0, self.multiplier + 0.1)
                    self.get_logger().info(f"Speed: {self.multiplier:.1f}")

        # 1. การควบคุมหุ่นยนต์หลัก (cmd_vel)
        move_msg = Twist()
        raw_linear = -self.joy.get_axis(1)
        raw_angular = -self.joy.get_axis(2) # เปลี่ยนเป็น Axis 3 (มักเป็นมาตรฐานขวา-ซ้าย ของจอยส่วนใหญ่)

        if abs(raw_linear) > self.deadzone:
            move_msg.linear.x = raw_linear * 0.6 * self.multiplier
        if abs(raw_angular) > self.deadzone:
            move_msg.angular.z = raw_angular * 0.6 * self.multiplier
        
        self.move_pub.publish(move_msg)

        # 2. การควบคุม Platform (platform_vel) โดยใช้ D-pad
        # get_hat(0) คืนค่าเป็น tuple (x, y) 
        # x: -1 (ซ้าย), 0 (กลาง), 1 (ขวา)
        # y: -1 (ลง), 0 (กลาง), 1 (ขึ้น)
        dpad = self.joy.get_hat(0)
        platform_msg = Twist()
        
        # เช็คค่าขึ้น/ลง (index 1 ของ tuple)
        if dpad[1] != 0:
            platform_msg.linear.x = float(dpad[1]) * 0.2 # ให้ความเร็ว platform คงที่ที่ 0.5
            self.get_logger().info(f"Platform Moving: {'UP' if dpad[1] > 0 else 'DOWN'}")
        else:
            platform_msg.linear.x = 0.0

        self.platform_pub.publish(platform_msg)

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # สั่งหยุดทุกอย่าง
        node.move_pub.publish(Twist())
        node.platform_pub.publish(Twist())
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()