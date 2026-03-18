#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
# import matplotlib.pyplot as plt  # เอากราฟออก
# from collections import deque    # ไม่ได้ใช้เก็บข้อมูลกราฟแล้ว
import time

class PIDManagerNode(Node):
    def __init__(self):
        super().__init__('pid_manager_node')
        
        # 1. Publisher สำหรับส่งค่า PID ไปยัง ESP32
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pid_parameters', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # ตัวแปรสำหรับคุมความเร็วการโชว์ Log (โชว์ทุก 5 วินาที)
        self.last_log_time = time.time()
        
        # 2. Subscriber รับค่าสถานะจาก ESP32 มาโชว์ใน Log
        self.subscription_drive = self.create_subscription(
            Float32MultiArray,
            'stageDrive', 
            self.drive_callback,
            10)
        
        # ตั้งค่า PID [P, I, D]
        # [Drive_L_P, I, D, Drive_R_P, I, D, Platform_P, I, D, Arm_P, I, D]
        self.pid_values = [
            30.0, 0.0, 1.0,   # Drive_L (ปรับ Ki ตามที่คุยกันเพื่อให้ถึง 0.3 จริง)
            31.25, 0.0, 1.0,   # Drive_R
            20.0, 0.0, 0.8,    # Platform
            10.0, 0.5, 1.0     # Arm
        ]

        # --- ส่วนกราฟเดิม (Comment Out) ---
        # self.max_points = 100
        # self.time_data = deque(maxlen=self.max_points)
        # self.out_l wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwddsssssssssssssssssssssssssssssssssssssssssd= deque(maxlen=self.max_points)
        # ... (ส่วนอื่นๆ ของกราฟถูกปิดใช้งาน)
        
        self.get_logger().info("=== PID Manager Started (Log Mode Only) ===")

    def timer_callback(self):
        # ส่งค่าไปยัง ESP32
        msg = Float32MultiArray()
        msg.data = self.pid_values
        self.publisher_.publish(msg)

        # แสดง Log ค่า PID ทุกๆ 5 วินาที
        current_time = time.time()
        if current_time - self.last_log_time > 5.0:
            self.show_pid_logs()
            self.last_log_time = current_time

    def show_pid_logs(self):
        """ แสดงค่า PID ที่ตั้งไว้ในปัจจุบัน """
        self.get_logger().info("-" * 45)
        self.get_logger().info("--- CURRENT PID SETTINGS ---")
        self.get_logger().info(f"Drive L  | P: {self.pid_values[0]:.2f} I: {self.pid_values[1]:.2f} D: {self.pid_values[2]:.2f}")
        self.get_logger().info(f"Drive R  | P: {self.pid_values[3]:.2f} I: {self.pid_values[4]:.2f} D: {self.pid_values[5]:.2f}")
        self.get_logger().info(f"Platform | P: {self.pid_values[6]:.2f} I: {self.pid_values[7]:.2f} D: {self.pid_values[8]:.2f}")
        self.get_logger().info(f"Arm      | P: {self.pid_values[9]:.2f} I: {self.pid_values[10]:.2f} D: {self.pid_values[11]:.2f}")
        self.get_logger().info("-" * 45)

    def drive_callback(self, msg):
        """ รับค่า Real-time จากหุ่นยนต์มาโชว์ใน Log """
        if len(msg.data) >= 4:
            # โชว์ค่าปัจจุบันเปรียบเทียบกับเป้าหมาย (โชว์ทุกครั้งที่ได้รับข้อมูล หรือจะตั้งเวลาหน่วงก็ได้)
            out_l, out_r = msg.data[0], msg.data[1]
            set_l, set_r = msg.data[2], msg.data[3]
            
            # แสดง Log สถานะการวิ่งจริง (Optional: ถ้ามันรกเกินไปสามารถใส่ if time.time() แบบข้างบนได้ครับ)
            self.get_logger().info(f"L: [Set {set_l:.2f} | Real {out_l:.2f}]  R: [Set {set_r:.2f} | Real {out_r:.2f}]")

    # def update_plot(self):
    #     pass # ปิดฟังก์ชันกราฟ

def main(args=None):
    rclpy.init(args=args)
    node = PIDManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        # plt.close('all') # ปิดส่วนกราฟ
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()