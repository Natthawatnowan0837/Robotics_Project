#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from collections import deque

class PIDManagerNode(Node):
    def __init__(self):
        super().__init__('pid_manager_node')
        
        # 1. Publisher สำหรับส่งค่า PID ไปยัง ESP32
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pid_parameters', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 2. Subscriber (ปิดการดึงข้อมูล balance ตามคำขอ)
        # self.subscription_bal = self.create_subscription(Float32MultiArray, 'balance', self.balance_callback, 10)
        
        self.subscription_drive = self.create_subscription(
            Float32MultiArray,
            'stageDrive', 
            self.drive_callback,
            10)
        
        # ตั้งค่า PID [P, I, D]
        self.pid_values = [
            1.0, 0.0, 0.0,   # Drive_L (ลองเริ่มที่ 10 ตามที่คุณถาม)
            1.0, 0.0, 0.0,   # Drive_R
            20.0, 0.0, 0.8,   # Platform
            10.0, 0.5, 1.0    # Arm
        ]

        # --- ส่วนการเตรียม Plot กราฟ ---
        self.max_points = 100  # จำนวนจุดที่จะแสดงบนกราฟ
        self.time_data = deque(maxlen=self.max_points)
        self.out_l = deque(maxlen=self.max_points)
        self.out_r = deque(maxlen=self.max_points)
        self.set_l = deque(maxlen=self.max_points)
        self.set_r = deque(maxlen=self.max_points)
        self.count = 0

        # สร้าง Figure สำหรับ Matplotlib
        plt.ion() # เปิดโหมด Interactive
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 8))
        self.get_logger().info("=== PID Manager Started (Plotting stageDrive) ===")

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.pid_values
        self.publisher_.publish(msg)

    def drive_callback(self, msg):
        """
        รับค่าจาก ESP32: [0]=OutL, [1]=OutR, [2]=SetL, [3]=SetR
        ตามที่เขียนไว้ในโค้ดฝั่ง Arduino
        """
        if len(msg.data) >= 4:
            self.count += 1
            self.time_data.append(self.count)
            self.out_l.append(msg.data[0])
            self.out_r.append(msg.data[1])
            self.set_l.append(msg.data[2])
            self.set_r.append(msg.data[3])
            
            self.update_plot()

    def update_plot(self):
        self.ax1.clear()
        self.ax2.clear()

        # กราฟล้อซ้าย
        self.ax1.plot(self.time_data, self.out_l, label='Output L (PWM)', color='red')
        self.ax1.plot(self.time_data, self.set_l, label='Setpoint L', color='blue', linestyle='--')
        self.ax1.set_title("Left Wheel Performance")
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True)

        # กราฟล้อขวา
        self.ax2.plot(self.time_data, self.out_r, label='Output R (PWM)', color='green')
        self.ax2.plot(self.time_data, self.set_r, label='Setpoint R', color='darkorange', linestyle='--')
        self.ax2.set_title("Right Wheel Performance")
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True)

        plt.tight_layout()
        plt.pause(0.001) # ขยับกราฟ

    # def balance_callback(self, msg):
    #     pass # ปิดการทำงานส่วนนี้ไว้

def main(args=None):
    rclpy.init(args=args)
    node = PIDManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()