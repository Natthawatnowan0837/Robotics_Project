#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import json
import os
from ament_index_python.packages import get_package_share_directory

class FinalTaskProcessor(Node):
    def __init__(self):
        super().__init__('final_task_processor')

        # --- Variables ---
        self.target_room_name = None
        self.room_coordinate = None
        self.target_floor = None
        self.rooms_dict = {}
        
        # ตัวแปรสถานะหลัก
        self.last_known_floor = 0.0
        self.current_mode = 1.0  # 0: Map, 1: Localize
        self.current_way = 1.0   # 0: Go, 1: Back (ค่าเริ่มต้น)

        # --- Subscriptions ---
        # รับชื่อห้อง
        self.sub_room = self.create_subscription(
            String, '/room_target', self.room_callback, 10)
        
        # รับสถานะชั้นและลิฟต์ [floor, status]
        self.sub_update_floor = self.create_subscription(
            Float32MultiArray, '/update_floor', self.update_floor_callback, 10)
        
        # รับค่า way ที่ถูกแก้ (Toggle) มาจาก Node CheckPosition
        self.sub_final_goal = self.create_subscription(
            Float32MultiArray, '/final_goal', self.final_goal_callback, 10)

        # --- Publishers ---
        self.pub_check_floor = self.create_publisher(Float32MultiArray, '/check_floor', 10)
        self.pub_goal = self.create_publisher(Float32MultiArray, 'pub_goal', 10)
        self.pub_manager = self.create_publisher(Float32MultiArray, '/my_manager_data', 10)
        
        # Load ข้อมูลจาก JSON
        self.load_rooms_data()
        self.get_logger().info("✅ Final Task Processor Started and Ready.")

    def load_rooms_data(self):
        try:
            pkg_share = get_package_share_directory('my_voice_control')
            json_path = os.path.join(pkg_share, 'commands.json')
            with open(json_path, 'r', encoding='utf-8') as f:
                self.rooms_dict = json.load(f).get("rooms", {})
            self.get_logger().info(f"Successfully loaded {len(self.rooms_dict)} rooms.")
        except Exception as e:
            self.get_logger().error(f"❌ Load JSON Error: {e}")

    def room_callback(self, msg):
        """ รับชื่อห้องและแจ้งระบบเช็คชั้น """
        name = msg.data
        if name in self.rooms_dict:
            self.target_room_name = name
            room_info = self.rooms_dict[name]
            self.room_coordinate = room_info.get('go')
            self.target_floor = room_info.get('floor')

            if self.target_floor is not None:
                floor_msg = Float32MultiArray()
                floor_msg.data = [float(self.target_floor)]
                self.pub_check_floor.publish(floor_msg)
                self.get_logger().info(f"🎯 Target set to: {name} on Floor {self.target_floor}")
        else:
            self.get_logger().warn(f"⚠️ Room {name} not found in database")

    def get_update_target(self, current_f, status):
        """ ตัดสินใจเลือกเป้าหมายตามสถานะลิฟต์ (0=ถึงแล้ว, 1=ขึ้น, -1=ลง) """
        target_name = None
        target_coords = None

        if status == 0.0:
            if self.target_floor is not None and int(current_f) == int(self.target_floor):
                target_name = self.target_room_name
                target_coords = self.room_coordinate
        
        elif status == 1.0:
            stair_key = f"Up_Stair{int(current_f)}"
            if stair_key in self.rooms_dict:
                target_name = stair_key
                target_coords = self.rooms_dict[stair_key].get('go')

        elif status == -1.0:
            stair_key = f"Down_Stair{int(current_f)}"
            if stair_key in self.rooms_dict:
                target_name = stair_key
                target_coords = self.rooms_dict[stair_key].get('go')

        return target_name, target_coords

    def update_floor_callback(self, msg):
        """ เมื่อได้รับอัปเดตชั้นปัจจุบันและสถานะลิฟต์ """
        if len(msg.data) < 2:
            return

        current_f = float(msg.data[0])
        status = msg.data[1]
        
        # เก็บค่าชั้นล่าสุดไว้ใช้ในฟังก์ชันอื่น
        self.last_known_floor = current_f

        target_name, target_coords = self.get_update_target(current_f, status)

        if target_name and target_coords:
            self.log_update_status(target_name, target_coords, current_f, status)
            
            # 1. ส่งข้อมูลให้ Manager เริ่มต้น
            self.publish_manager_data()

            # 2. ส่งเป้าหมายพิกัดให้ CheckPosition ไปตรวจสอบ X-Axis
            goal_msg = Float32MultiArray()
            try:
                if isinstance(target_coords, list):
                    goal_msg.data = [float(target_coords[0]), float(target_coords[1]), float(self.current_way)]
                elif isinstance(target_coords, dict):
                    goal_msg.data = [float(target_coords.get('x', 0)), float(target_coords.get('y', 0)), float(self.current_way)]
                
                self.pub_goal.publish(goal_msg)
            except Exception as e:
                self.get_logger().error(f"❌ Error processing coordinates: {e}")

    def final_goal_callback(self, msg):
        """ 
        รับพิกัดที่ยืนยันแล้วจาก CheckPosition 
        และอัปเดตค่า Way (ทิศทาง) ใหม่หากมีการเปลี่ยนแปลง 
        """
        if len(msg.data) >= 3:
            new_way = float(msg.data[2])
            
            # ถ้าค่า Way เปลี่ยนไป (เช่น จาก Go เป็น Back) ให้ส่งข้อมูลให้ Manager ใหม่
            if self.current_way != new_way:
                self.get_logger().info(f"🔄 Way updated from CheckPosition: {self.current_way} -> {new_way}")
                self.current_way = new_way
                
                # แจ้ง Manager ทันทีเพื่อให้ระบบเปลี่ยนโหมดการเดิน
                self.publish_manager_data()

    def publish_manager_data(self):
        """ ฟังก์ชันกลางสำหรับส่งข้อมูลไปที่ /my_manager_data """
        manager_msg = Float32MultiArray()
        manager_msg.data = [float(self.current_mode), float(self.current_way), float(self.last_known_floor)]
        self.pub_manager.publish(manager_msg)
        self.get_logger().info(f"📤 Sent to Manager: Mode={self.current_mode}, Way={self.current_way}, Floor={self.last_known_floor}")

    def log_update_status(self, name, coords, floor, status):
        self.get_logger().info("-" * 30)
        self.get_logger().info(f"📍 UPDATE TARGET: {name}")
        self.get_logger().info(f"🌐 COORDINATE: {coords}")
        self.get_logger().info(f"🏢 CURRENT FLOOR: {floor} | STATUS: {status}")
        self.get_logger().info("-" * 30)

def main(args=None):
    rclpy.init(args=args)
    node = FinalTaskProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()