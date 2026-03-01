#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import os

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        
        # ป้องกัน Error เมื่อรันผ่าน SSH (Headless mode)
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        
        # Publisher สำหรับตัวหุ่น และ Platform
        self.move_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.platform_pub = self.create_publisher(Twist, 'platform_cmd_vel', 10)
        
        # การตั้งค่าความเร็ว
        self.base_speed = 1.0     # ปรับเป็น 1.0 ตามที่ต้องการ
        self.multiplier = 1.0     # ตัวคูณจากการกดปุ่ม A/Y
        self.deadzone = 0.15      # ค่ากันจอยเดินเอง
        
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("--- ไม่พบจอย Xbox! กรุณาเชื่อมต่อจอย ---")
            return
            
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        
        self.get_logger().info("---------------------------------------------")
        self.get_logger().info(f"Base Speed ตั้งไว้ที่: {self.base_speed}")
        self.get_logger().info("A: ลด Speed | Y: เพิ่ม Speed")
        self.get_logger().info("Analog ซ้าย/ขวา: ควบคุมหุ่น (cmd_vel)")
        self.get_logger().info("D-pad ขึ้น/ลง: ควบคุม Platform (platform_cmd_vel)")
        self.get_logger().info("---------------------------------------------")

        # รัน Loop ทุกๆ 0.05 วินาที (20Hz)
        self.timer = self.create_timer(0.05, self.controller_loop)

    def controller_loop(self):
        # ตรวจสอบ Event จากปุ่มกด
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0: # ปุ่ม A
                    self.multiplier = max(0.1, self.multiplier - 0.1)
                    self.get_logger().info(f"ความเร็วปัจจุบัน: {self.multiplier * self.base_speed:.2f}")
                elif event.button == 3: # ปุ่ม Y
                    self.multiplier = min(2.0, self.multiplier + 0.1)
                    self.get_logger().info(f"ความเร็วปัจจุบัน: {self.multiplier * self.base_speed:.2f}")

        # 1. ควบคุมหุ่นยนต์หลัก (cmd_vel)
        move_msg = Twist()
        
        # อ่านค่าจากจอย (Axis 1 = หน้า/หลัง, Axis 2 = ซ้าย/ขวา)
        raw_linear = -self.joy.get_axis(1)
        raw_angular = -self.joy.get_axis(2) 

        # คำนวณความเร็วเชิงเส้น (Linear X)
        if abs(raw_linear) > self.deadzone:
            # คำนวณค่า: ค่าจอย * 1.0 * Multiplier
            val_x = raw_linear * self.base_speed * self.multiplier
            move_msg.linear.x = float(val_x)

        # คำนวณความเร็วการหมุน (Angular Z)
        if abs(raw_angular) > self.deadzone:
            val_z = raw_angular * self.base_speed * self.multiplier
            move_msg.angular.z = float(val_z)
        
        self.move_pub.publish(move_msg)

        # 2. ควบคุม Platform (platform_cmd_vel) โดยใช้ D-pad
        dpad = self.joy.get_hat(0)
        platform_msg = Twist()
        
        # dpad[1] คือ ขึ้น(1) / ลง(-1)
        if dpad[1] != 0:
            platform_msg.linear.x = float(dpad[1]) * 0.2
            # แสดง log เฉพาะตอนกด
            # self.get_logger().info(f"Platform: {'UP' if dpad[1] > 0 else 'DOWN'}")
        
        self.platform_pub.publish(platform_msg)

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("กำลังปิดระบบ...")
    finally:
        # สั่งหยุดหุ่นยนต์และ Platform ก่อนปิดโปรแกรม
        stop_msg = Twist()
        node.move_pub.publish(stop_msg)
        node.platform_pub.publish(stop_msg)
        
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()