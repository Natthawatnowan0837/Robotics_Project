#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray
import os
import sqlite3 # สมมติว่า go.db เป็น SQLite ถ้าเป็นอย่างอื่นให้เปลี่ยน library ครับ

class My_manager(Node):
    def __init__(self):
        super().__init__('my_manager')
        
        # เก็บ Path หลักของ package ไว้
        self.package_share = get_package_share_directory('my_manager')

        # Subscribe ข้อมูลเซนเซอร์/ชั้น
        self.sub_sensors = self.create_subscription(
            Float32MultiArray,
            'check_floor',
            self.check_floor_callback,
            10)

        self.get_logger().info("🚀 My Manager Node started...")

    def check_floor_callback(self, msg):
        # สมมติว่า msg.data[0] คือเลขชั้น (floor_goal)
        if len(msg.data) >= 1:
            floor_goal = int(msg.data[0])
            self.get_logger().info(f"📍 กำลังประมวลผลชั้น: {floor_goal}")
            
            # เรียกฟังก์ชันเปิด Database
            self.open_database(floor_goal)

    def open_database(self, floor_num):
        db_path = os.path.join(self.package_share, 'maps', f'floor{floor_num}', 'go.db')

        if os.path.exists(db_path):
            try:

                conn = sqlite3.connect(db_path)
                self.get_logger().info(f"✅ เปิด Database สำเร็จ: {db_path}")
                conn.close()
            except Exception as e:
                self.get_logger().error(f"❌ ไม่สามารถเปิด Database ได้: {str(e)}")
        else:
            self.get_logger().warn(f"⚠️ ไม่พบไฟล์ Database ที่: {db_path}")

def main(args=None):
    rclpy.init(args=args)
    node = My_manager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 ปิดระบบ My Manager")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()