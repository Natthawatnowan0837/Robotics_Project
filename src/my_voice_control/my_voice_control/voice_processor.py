#!/usr/bin/env python3
import rclpy
import json
import os
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# =============================================================
# 1. IMPORT ให้ถูก Package
# ตรวจสอบว่า Sendposition.srv อยู่ใน package ไหน (my_command หรือ my_control?)
# ถ้าอยู่ใน my_command ให้ใช้บรรทัดนี้:
# =============================================================
from my_command.srv import Sendposition 

pkg_share = get_package_share_directory('my_voice_control')
json_path = os.path.join(pkg_share, 'commands.json')

with open(json_path, 'r', encoding='utf-8') as f:
    commands_json = json.load(f)
rooms_dict = commands_json["rooms"]

class VoiceProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Subscriber รับเสียง
        self.create_subscription(String, '/voice_cmd', self.voice_cmd_callback, 10)

        # 2. สร้าง Service Client โดยใช้ Type 'Sendposition'
        self.cli = self.create_client(Sendposition, 'move_robot_service')

        # รอให้ Server (หุ่นยนต์) ออนไลน์
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service /move_robot_service not available, waiting...')

        # self.get_logger().info("Voice Processor Ready (Service Mode with Z-Floor)")

    def respond_room(self, action, room):
        if action and room in rooms_dict:
            # คืนค่า list เช่น [2, 0, 2]
            return rooms_dict[room]["position"]
        return None

    def voice_cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            room = data.get("room")
            action = data.get("action")
            
            # ได้ค่า position เป็น list [x, y, z]
            position = self.respond_room(action, room)

            if position and len(position) >= 3:
                # ====================================================
                # 3. แก้ไขจุดสำคัญ: เปลี่ยนจาก MoveToRoom เป็น Sendposition
                # ====================================================
                req = Sendposition.Request()
                
                req.room_name = str(room)
                req.x = float(position[0]) # ค่า index 0
                req.y = float(position[1]) # ค่า index 1
                req.z = float(position[2]) # ค่า index 2 (ชั้น)

                # 4. ส่ง Request แบบ Async (Call Service)
                self.future = self.cli.call_async(req)
                self.future.add_done_callback(self.response_callback)

                self.get_logger().info(f"Sent request: Room {room} at ({req.x}, {req.y}) Floor {req.z}")
                
                # Feedback เสียงพูด
                os.system(f'espeak -vth "กำลังไปที่ {room} ชั้น {int(req.z)}" 2>/dev/null &')
            
            elif position:
                 self.get_logger().warn(f"Position data incomplete (needs x,y,z): {position}")


        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    # 5. ฟังก์ชันจัดการเมื่อได้รับคำตอบจาก Robot (Response)
    def response_callback(self, future):
        try:
            response = future.result()
            # เช็คค่าที่ Robot ตอบกลับมา (bool success, string message)
            if response.success:
                self.get_logger().info(f"✅ Robot accepted: {response.message}")
            else:
                self.get_logger().warn(f"❌ Robot rejected: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()