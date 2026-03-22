#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
# แก้ไข: เปลี่ยนจาก Int32 เป็น Int32MultiArray
from std_msgs.msg import Float32MultiArray, Int32MultiArray 
import os

class My_manager(Node):
    def __init__(self):
        super().__init__('my_manager')
        
        self.package_share = get_package_share_directory('my_manager')

        # 1. Subscribe ข้อมูลเซนเซอร์/ชั้น
        self.sub_sensors = self.create_subscription(
            Float32MultiArray,
            'check_floor',
            self.check_floor_callback,
            10)

        # 2. Publisher: เปลี่ยน Topic 'active' ให้ส่งเป็น Int32MultiArray
        self.pub_active_localize = self.create_publisher(
            Int32MultiArray,
            'active',
            10)

        self.get_logger().info("🚀 My Manager Node started (Array Mode)...")

    def check_floor_callback(self, msg):
        if len(msg.data) >= 4:
            target_x  = msg.data[0]
            target_y  = msg.data[1]
            current_f = msg.data[2]
            status    = msg.data[3]
            
            # --- แก้ไข: ส่งค่าเป็น [check_localize, 1] ---
            # สมมติว่าค่า check_localize คือค่า status หรือค่าที่คุณกำหนดไว้ 
            # ในที่นี้ผมใส่เป็นตัวอย่างให้เห็นโครงสร้าง [ค่าสถานะ, 1]
            active_msg = Int32MultiArray()
            
            # ใส่ข้อมูลลงใน list (ตัวอย่าง: [int(status), 1])
            # คุณสามารถเปลี่ยน int(status) เป็นตัวแปรอื่นที่ต้องการได้
            active_msg.data = [int(status), 1] 
            
            self.pub_active_localize.publish(active_msg)
            # ---------------------------------------

            status_text = ""
            if status == 0.0:
                status_text = "✅ ถึงชั้นเป้าหมายแล้ว (MOVE TO TARGET)"
            elif status == 1.0:
                status_text = "🔼 ต้องขึ้นบันได (GO TO STAIR UP)"
            elif status == -1.0:
                status_text = "🔽 ต้องลงบันได (GO TO STAIR DOWN)"

            self.get_logger().info("---------------------------------------")
            # แสดงค่าที่ส่งออกไปใน Logger
            self.get_logger().info(f"📤 Sent to 'active': {active_msg.data}") 
            self.get_logger().info(f"📍 [Floor Status]: {status_text}")
            self.get_logger().info(f"🏠 [Current Floor]: {int(current_f)}")
            self.get_logger().info(f"🎯 [Target Pose]: X={target_x:.2f}, Y={target_y:.2f}")
            self.get_logger().info("---------------------------------------")
            
        else:
            self.get_logger().warn(f"⚠️ ข้อมูลจาก check_floor ไม่ครบ", throttle_duration_sec=2.0)

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