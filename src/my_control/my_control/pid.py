#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 # ลองเปลี่ยนเป็น Float32 ตามมาตรฐานทั่วไป

class MotorSpeedLogger(Node):
    def __init__(self):
        super().__init__('motor_speed_logger_node')
        
        # เพิ่มการแจ้งเตือนว่า Node กำลังรอข้อมูล
        self.get_logger().info("Waiting for data on /motor_left_rps and /motor_right_rps...")

        self.sub_left = self.create_subscription(
            Float32, 
            '/motor_left_rps',
            self.left_callback,
            10)
            
        self.sub_right = self.create_subscription(
            Float32,
            '/motor_right_rps',
            self.right_callback,
            10)

    def left_callback(self, msg):
        # พิมพ์ค่าทันทีที่ได้รับ
        print(f"DEBUG: Left Received {msg.data}") # ใช้ print ธรรมดาช่วยเช็คด้วย
        self.get_logger().info(f"Left Motor: {msg.data:.4f} RPS")

    def right_callback(self, msg):
        self.get_logger().info(f"Right Motor: {msg.data:.4f} RPS")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()