#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')

        # 1. เปลี่ยนจาก Service มาเป็น Subscriber รอรับ JSON จาก wave_to_text
        self.sub_target = self.create_subscription(
            String,
            '/robot_target',
            self.target_callback,
            10
        )
        # 2. Publisher สำหรับส่งพิกัดให้ Nav2 (หรือระบบ Navigation ของคุณ)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        # จำลองเซ็นเซอร์ว่าตอนนี้หุ่นอยู่ชั้นไหน (คุณสามารถเอา Topic เซ็นเซอร์จริงมาผูกทีหลังได้)
        self.current_floor = 2.0

        self.get_logger().info('✅ Manager Node พร้อมทำงานแล้ว (รอรับเป้าหมายจาก /robot_target)...')

    def target_callback(self, msg):
        try:
            # --- ถอดรหัส JSON ที่ส่งมาจากโหนดเสียง ---
            data = json.loads(msg.data)
            room_name = data.get("room_name", "Unknown")
            target_x = data.get("x", 0.0)
            target_y = data.get("y", 0.0)
            target_z = data.get("z", 0.0)       # ชั้นเป้าหมาย
            map_file = data.get("map", None)    # แผนที่ที่ต้องใช้ (ถ้ามี)

            # --- Logger Info ---
            self.get_logger().info('>>> [ได้รับเป้าหมายใหม่จากเสียง] <<<')
            self.get_logger().info(f'ชื่อสถานที่: {room_name} | พิกัด X: {target_x:.2f} Y: {target_y:.2f} ชั้น: {target_z}')
            if map_file:
                self.get_logger().info(f'🗺️ ห้องนี้ผูกกับแผนที่: {map_file}')

            # =========================================================
            # [ลอจิกการตัดสินใจ - Decision Making]
            # =========================================================
            if target_z != self.current_floor:
                # กรณีคนละชั้น
                self.get_logger().warn(f'⚠️ ไม่สามารถไปตรงๆ ได้! หุ่นอยู่ชั้น {self.current_floor} แต่เป้าหมายอยู่ชั้น {target_z}')
                self.get_logger().info('👉 กำลังเปลี่ยนพิกัดเป้าหมายไปยัง "หน้าลิฟต์"...')
                # TODO: คุณสามารถกำหนดพิกัดหน้าลิฟต์แทนที่เป้าหมายเดิมได้ตรงนี้
                return # หยุดการทำงานชั่วคราว ไม่ให้ส่งพิกัดห้องไปวิ่งจริง

            # --- ถ้าอยู่ชั้นเดียวกัน สร้าง Message เพื่อ Publish ไปยัง /goal ---
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = float(target_x)
            goal_msg.pose.position.y = float(target_y)
            goal_msg.pose.position.z = 0.0 # สำหรับ Nav2 แกน Z ของการวิ่งมักจะเป็น 0 เสมอ
            goal_msg.pose.orientation.w = 1.0 # หน้าตรงเสมอ

            # Publish ข้อมูลออกไป
            self.goal_pub.publish(goal_msg)
            self.get_logger().info(f'🚀 Published goal for {room_name} to /goal topic')

        except json.JSONDecodeError:
            self.get_logger().error("❌ JSON Decode Error: ข้อมูลที่ส่งมาอ่านไม่ออก")
        except Exception as e:
            self.get_logger().error(f"❌ Error in target_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()