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
        
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        
        # Publisher สำหรับตัวหุ่น, Platform และ Arm
        self.move_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.platform_pub = self.create_publisher(Twist, 'platform_cmd_vel', 10)
        self.arm_pub = self.create_publisher(Twist, 'arm_cmd_vel', 10) # เพิ่ม Publisher สำหรับแขน
        
        self.base_speed = 1.0     
        self.multiplier = 1.0     
        self.deadzone = 0.15      
        
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("--- ไม่พบจอย Xbox! ---")
            return
            
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        
        self.get_logger().info("---------------------------------------------")
        self.get_logger().info("RT: Arm UP (+1) | LT: Arm DOWN (-1)")
        self.get_logger().info("D-pad: Platform | Analog: Robot Move")
        self.get_logger().info("---------------------------------------------")

        self.timer = self.create_timer(0.05, self.controller_loop)

    def controller_loop(self):
        # ตรวจสอบ Event ปุ่ม A/Y (เหมือนเดิม)
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0: # A
                    self.multiplier = max(0.1, self.multiplier - 0.1)
                elif event.button == 3: # Y
                    self.multiplier = min(2.0, self.multiplier + 0.1)

        # --- 1. ควบคุมหุ่นยนต์หลัก (cmd_vel) ---
        move_msg = Twist()
        raw_linear = -self.joy.get_axis(1)
        raw_angular = -self.joy.get_axis(2) 

        if abs(raw_linear) > self.deadzone:
            move_msg.linear.x = float(raw_linear * self.base_speed * self.multiplier)
        if abs(raw_angular) > self.deadzone:
            move_msg.angular.z = float(raw_angular * self.base_speed * self.multiplier * 1.5)
        self.move_pub.publish(move_msg)

        # --- 2. ควบคุม Platform (platform_cmd_vel) ---
        dpad = self.joy.get_hat(0)
        platform_msg = Twist()
        if dpad[1] != 0:
            platform_msg.linear.x = float(dpad[1]) * 0.2
        self.platform_pub.publish(platform_msg)

        # --- 3. ควบคุม Arm (arm_vel) ด้วย RT และ LT ---
        arm_msg = Twist()
        
        # อ่านค่า Trigger (ค่าปกติของ pygame คือ -1.0 ถึง 1.0)
        # หมายเหตุ: Index ของ Axis อาจต่างกันตาม Driver (RT ปกติคือ 5, LT คือ 2 หรือ 4)
        rt_val = self.joy.get_axis(5) 
        lt_val = self.joy.get_axis(4) 

        # แปลงค่าจาก (-1.0 ถึง 1.0) ให้เป็น (0.0 ถึง 1.0)
        rt_pressed = (rt_val + 1.0) / 2.0
        lt_pressed = (lt_val + 1.0) / 2.0

        # Logic: ถ้ากด RT ให้เป็นบวก, ถ้ากด LT ให้เป็นลบ
        if rt_pressed > 0.1: # Threshold กันค่าแกว่ง
            arm_msg.linear.x = 1.0
        elif lt_pressed > 0.1:
            arm_msg.linear.x = -1.0
        else:
            arm_msg.linear.x = 0.0

        self.arm_pub.publish(arm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.move_pub.publish(stop_msg)
        node.platform_pub.publish(stop_msg)
        node.arm_pub.publish(stop_msg) # หยุดแขนด้วย
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()