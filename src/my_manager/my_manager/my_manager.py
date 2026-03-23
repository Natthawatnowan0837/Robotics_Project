#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String 

class MyManager(Node):  # เปลี่ยนชื่อคลาสให้เป็น CamelCase ตามสไตล์ Python
    def __init__(self):
        super().__init__('my_manager')
        
        # --- ตัวแปรภายใน ---
        self.mode = 0.0
        self.way = 0.0
        self.floor = 0.0
        self.x = 0.0  # สมมติค่าเริ่มต้น
        self.y = 0.0

        self.is_localized_finished = False
        self.is_nav_mode = False
        self.is_nav_active = False

        # --- Subscriptions ---
        # 1. รับข้อมูลการจัดการ (mode, way, floor)
        self.sub_manager_data = self.create_subscription(
            Float32MultiArray,
            '/my_manager_data',
            self.manager_data_callback,
            10)
        
        # 2. รับสถานะจากระบบ (localize, action, nav2)
        self.sub_status = self.create_subscription(
            String,
            'status',
            self.status_callback,
            10)

        # --- Publishers ---
        self.pub_action = self.create_publisher(String, 'action', 10)
        self.pub_opening = self.create_publisher(Float32MultiArray, 'opening', 10)
        self.pub_update_target = self.create_publisher(Float32MultiArray, 'update_target', 10)

        self.get_logger().info("🚀 My Manager Node started...")

    def manager_data_callback(self, msg):
            """ เมื่อได้รับข้อมูลใหม่ (เช่น Way เปลี่ยน) ให้ Reset สถานะทั้งหมดเพื่อเริ่มใหม่ """
            if len(msg.data) >= 3:
                # ตรวจสอบก่อนว่าข้อมูลเปลี่ยนจริงไหม (ป้องกันการเริ่มใหม่ซ้ำซากถ้าข้อมูลเดิมส่งมา)
                if self.mode != msg.data[0] or self.way != msg.data[1] or self.floor != msg.data[2]:
                    
                    self.mode = msg.data[0]
                    self.way = msg.data[1]
                    self.floor = msg.data[2]
                    
                    self.get_logger().info(f"📊 NEW DATA RECEIVED: Resetting State Machine to Start Localization...")

                    # --- [จุดสำคัญ: Reset ทุกอย่างเพื่อเริ่มใหม่หมด] ---
                    self.is_localized_finished = False
                    self.is_nav_mode = False
                    self.is_nav_active = False
                    # ------------------------------------------

                    # ส่งไปที่ opening เพื่อให้ LaunchSwitcher เปิดโหมด (Localize/Map) ใหม่ตาม Way ที่เปลี่ยน
                    opening_msg = Float32MultiArray()
                    opening_msg.data = [float(self.mode), float(self.way), float(self.floor)]
                    self.pub_opening.publish(opening_msg)
                else:
                    self.get_logger().info("📊 Received duplicate data, ignoring reset.")

    def status_callback(self, msg):
        """ รับค่าจาก Topic 'status' และเปลี่ยน State ของหุ่นยนต์ """
        received_status = msg.data.lower()
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.get_logger().info(f"📩 [RECEIVED STATUS]: {msg.data}")
        
        # 1. เช็ค Localize
        if 'localize,done' in received_status:
            self.is_localized_finished = True
            self.get_logger().info("🎯 Localization SUCCESS! Sending 'action'")
            self.send_action("action")

        # 2. เช็ค Action
        elif 'action,done' in received_status:
            self.is_nav_mode = True
            self.get_logger().info("🎯 Action SUCCESS! Sending 'position and nav2'")
            self.send_action("position")
            self.send_action("nav2")

        elif 'final,done' in received_status:
            self.is_nav_mode = True
            self.get_logger().info("🎯 Action SUCCESS! Sending 'nav2'")
            

        # 3. เช็ค Nav2
        elif 'nav2,done' in received_status:
            self.is_nav_active = True 
            self.get_logger().info("🏁 Navigator READY! Publishing update_target...")
            
            update_msg = Float32MultiArray()
            update_msg.data = [float(self.x), float(self.y)]
            self.pub_update_target.publish(update_msg)
            
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    def send_action(self, text):
        msg = String()
        msg.data = text
        self.pub_action.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 ปิดระบบ My Manager")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()