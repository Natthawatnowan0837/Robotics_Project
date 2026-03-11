#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
# เปลี่ยนจาก Float32MultiArray เป็น Twist
from geometry_msgs.msg import Twist 
import pygame
import os

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        
        self.declare_parameter('mode', 'default')
        current_mode = self.get_parameter('mode').get_parameter_value().string_value
        self.get_logger().info(f"--- Starting Node with Mode: {current_mode} ---")

        # --- [ Configuration ] ---
        if current_mode == 'map':
            self.cfg = {
                'linear_max':  10.0,
                'angular_max': 10.0,
                'deadzone': 0.1,
                'arm_speed': 0.5  # เพิ่มให้ครบเพื่อกัน Error
            }
        else:
            self.cfg = {
                'linear_max':  10.0,
                'angular_max': 10.0,
                'deadzone': 0.1,
                'arm_speed': 1.0
            }

        os.environ["SDL_VIDEODRIVER"] = "dummy"
        
        # เปลี่ยน Topic name เป็น 'cmd_vel' (มาตรฐาน ROS 2) และใช้ Twist
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("--- ไม่พบจอย Xbox! ---")
            return
            
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(f"--- Config Applied: Linear {self.cfg['linear_max']}, Angular {self.cfg['angular_max']} ---")
        self.timer = self.create_timer(0.05, self.controller_loop)

    def controller_loop(self):
        pygame.event.pump()
        
        # สร้าง Message object ชนิด Twist
        twist = Twist()

        # อ่านค่าจาก Joystick (Linear=แกน Y, Angular=แกน X ของจอยขวา/ซ้าย)
        raw_linear = -self.joy.get_axis(1)  # แกนเดินหน้า-ถอยหลัง
        raw_angular = -self.joy.get_axis(2) # แกนหมุนตัว
        
        # ใส่ค่าลงใน Twist (linear.x และ angular.z)
        if abs(raw_linear) > self.cfg['deadzone']:
            twist.linear.x = float(raw_linear * self.cfg['linear_max'])
            
        if abs(raw_angular) > self.cfg['deadzone']:
            twist.angular.z = float(raw_angular * self.cfg['angular_max'])

        # หมายเหตุ: Twist ไม่มีฟิลด์สำหรับ Arm หรือ D-pad โดยตรง 
        # หากต้องการส่งค่าแขนกลด้วย แนะนำให้ใช้ Topic แยก หรือใช้ Custom Message
        
        self.cmd_vel_pub.publish(twist)

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