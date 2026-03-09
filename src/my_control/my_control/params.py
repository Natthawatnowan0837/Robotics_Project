#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PIDManagerNode(Node):
    def __init__(self):
        super().__init__('pid_manager_node')
        
        # 1. Publisher สำหรับส่งค่า PID ไปยัง ESP32
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pid_parameters', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # ส่งทุก 0.1 วินาที
        
        # 2. Subscriber สำหรับรับค่าจาก ESP32 (Topic: balance)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'balance', 
            self.balance_callback,
            10)
        
        # กำหนดค่า PID ที่ต้องการส่ง (ปรับจูนที่นี่)
        # [P, I, D] ของแต่ละส่วน
        self.pid_values = [
            4.8, 0.0, 0.0,   # Drive_L
            5.0, 0.0, 0.0,   # Drive_R
            20.0, 0.0, 0.8,   # Platform (ที่จูนกันล่าสุด: Kp=0.2, Kd=0.8)
            10.0, 0.5, 1.0   # Arm
        ]

        self.get_logger().info("=== PID Manager Started (No Plotting) ===")

    def timer_callback(self):
        """ส่งค่า PID ออกไปอย่างต่อเนื่อง"""
        msg = Float32MultiArray()
        msg.data = self.pid_values
        self.publisher_.publish(msg)

    def balance_callback(self, msg):
        """รับค่าจาก ESP32 มาแสดงผลบน Terminal"""
        if len(msg.data) >= 2:
            # แสดงค่ามุม (index 0) และ PWM (index 1) ผ่าน Logger แทนการพล็อตกราฟ
            angle = msg.data[0]
            pwm = msg.data[1]
            self.get_logger().info(f"Angle: {angle:>6.2f} | PWM: {pwm:>6.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PIDManagerNode()

    try:
        # ใช้ rclpy.spin ตรงๆ ไม่ต้องแยก Thread เพราะไม่มีงาน GUI แล้ว
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
##commit