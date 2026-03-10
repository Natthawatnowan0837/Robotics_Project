#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pygame
import os

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        
        # 1. ประกาศ Parameter 'mode' และตั้งค่า default เป็น 'default'
        self.declare_parameter('mode', 'default')
        
        # 2. ดึงค่าจาก Parameter 'mode' มาตรวจสอบ
        current_mode = self.get_parameter('mode').get_parameter_value().string_value
        self.get_logger().info(f"--- Starting Node with Mode: {current_mode} ---")

        # --- [ ส่วนตั้งค่าความเร็ว ] ---
        if current_mode == 'map':
            # ค่าเมื่อรับ parameter 'map'
            self.cfg = {
                'linear_max':  1.5,    # แก้ตามที่คุณต้องการ
                'angular_max': 1.5,    # แก้ตามที่คุณต้องการ
                'platform_speed': 0.5,
                'arm_speed': 0.5,
                'deadzone': 0.15
            }
        else:
            # ค่าปกติ (Default)
            self.cfg = {
                'linear_max':  15.0,
                'angular_max': 15.0,
                'platform_speed': 0.5,
                'arm_speed': 0.5,
                'deadzone': 0.15
            }
        # ---------------------------------------

        os.environ["SDL_VIDEODRIVER"] = "dummy"
        self.ctrl_pub = self.create_publisher(Float32MultiArray, 'controller', 10)
        
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
        msg = Float32MultiArray()
        ctrl_data = [0.0, 0.0, 0.0, 0.0]

        # คำนวณความเร็วหุ่นยนต์
        raw_linear = -self.joy.get_axis(1)
        raw_angular = -self.joy.get_axis(2) 
        
        if abs(raw_linear) > self.cfg['deadzone']:
            ctrl_data[0] = float(raw_linear * self.cfg['linear_max'])
            
        if abs(raw_angular) > self.cfg['deadzone']:
            ctrl_data[1] = float(raw_angular * self.cfg['angular_max'])

        # ส่วน D-pad และ Arm คงเดิม...
        dpad = self.joy.get_hat(0)
        if dpad[1] != 0:
            ctrl_data[2] = float(dpad[1] * self.cfg['platform_speed'])

        rt_val = (self.joy.get_axis(5) + 1.0) / 2.0
        lt_val = (self.joy.get_axis(4) + 1.0) / 2.0

        if rt_val > 0.2:
            ctrl_data[3] = float(self.cfg['arm_speed'])
        elif lt_val > 0.2:
            ctrl_data[3] = float(-self.cfg['arm_speed'])

        msg.data = ctrl_data
        self.ctrl_pub.publish(msg)

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