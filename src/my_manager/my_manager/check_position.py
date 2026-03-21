#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray
import joblib
import os
import numpy as np

class Check_position(Node):
    def __init__(self):
        super().__init__('Check_position')
        
        # 1. โหลด Model และ Scaler
        package_name = 'my_manager' 
        # 2. Subscribe ข้อมูลเซนเซอร์ (Real-time)
        self.sub_sensors = self.create_subscription(
            Float32MultiArray,
            'check_floor',
            self.check_floor_callback,
            10)

        self.current_floor = None

    def target_callback(self, msg):
        if len(msg.data) >= 3 and self.current_floor is not None:
            x_goal = float(msg.data[0])
            y_goal = float(msg.data[1])
            self.get_logger().info(f"🎯 ตำแหน่ง target : {x_goal,y_goal}")

    def check_floor_callback(self, msg):
        if len(msg.data) >= 3 and self.current_floor is not None:
            floor_goal = float(msg.data[0])
            floor_status = float(msg.data[1])
            self.get_logger().info(f"🎯 ชั้น : {floor_goal,floor_status}")
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Check_position()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 ปิดระบบ AI Predictor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()