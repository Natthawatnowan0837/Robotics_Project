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
        
        # --- [ ส่วนที่แก้ค่าความเร็วได้ง่ายๆ ] ---
        self.cfg = {
            'linear_max':  2.0,    # ความเร็วเดินหน้า-ถอยหลังสูงสุด
            'angular_max': 3.0,    # ความเร็วการเลี้ยวสูงสุด
            'platform_speed': 0.5, # ความเร็วเวลายก Platform (ขึ้น/ลง)
            'arm_speed': 0.5,      # ความเร็วเวลายก Arm
            'deadzone': 0.15,      # ค่ากันจอยเดินเอง (0.0 - 1.0)
            'boost_multiplier': 1.5 # ตัวคูณความเร็วเวลาอยากให้วิ่งเร็วพิเศษ (ถ้าจะทำเพิ่ม)
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
        self.get_logger().info("--- Xbox Controller Node Started (Speed Configurable) ---")
        self.timer = self.create_timer(0.05, self.controller_loop)

    def controller_loop(self):
        pygame.event.pump()
        msg = Float32MultiArray()
        
        # ลำดับข้อมูล: [0]=Linear, [1]=Angular, [2]=Platform, [3]=Arm
        ctrl_data = [0.0, 0.0, 0.0, 0.0]

        # 1. คำนวณความเร็วหุ่นยนต์ (ใช้ค่าจาก self.cfg)
        raw_linear = -self.joy.get_axis(1)
        raw_angular = -self.joy.get_axis(2) 
        
        # ใช้ Max Speed จาก Config
        if abs(raw_linear) > self.cfg['deadzone']:
            ctrl_data[0] = float(raw_linear * self.cfg['linear_max'])
            
        if abs(raw_angular) > self.cfg['deadzone']:
            ctrl_data[1] = float(raw_angular * self.cfg['angular_max'])

        # 2. ควบคุม Platform (D-pad)
        dpad = self.joy.get_hat(0)
        if dpad[1] != 0:
            # ส่งค่าความเร็วตามที่ตั้งไว้ใน Config
            ctrl_data[2] = float(dpad[1] * self.cfg['platform_speed'])

        # 3. ควบคุม Arm (RT/LT)
        # RT (Axis 5), LT (Axis 4) - ค่าดิบจะได้ -1.0 ถึง 1.0
        rt_val = (self.joy.get_axis(5) + 1.0) / 2.0
        lt_val = (self.joy.get_axis(4) + 1.0) / 2.0

        if rt_val > 0.2:
            ctrl_data[3] = float(self.cfg['arm_speed'])
        elif lt_val > 0.2:
            ctrl_data[3] = float(-self.cfg['arm_speed'])

        # ส่งข้อมูลเข้า Topic
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
        # ส่งค่า 0 เพื่อหยุดหุ่นยนต์
        stop_msg = Float32MultiArray()
        stop_msg.data = [0.0, 0.0, 0.0, 0.0]
        node.ctrl_pub.publish(stop_msg)
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()