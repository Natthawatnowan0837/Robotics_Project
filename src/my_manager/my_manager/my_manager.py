import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from enum import Enum
import json
import os

from ament_index_python.packages import get_package_share_directory
from my_command.srv import CheckFloor
from my_command.srv import OpenMap # อย่าลืม import service ตัวใหม่
from my_command.srv import CheckLocalize
from my_command.srv import CheckPosition
from my_command.srv import Nav2

class RobotState(Enum):
    IDLE = 0          
    CHECK_FLOOR = 1  
    OPEN_MAP = 2      
    CHECK_LOCALIZE = 3     
    CHECK_POSITION = 4   
    NAV2 = 5         

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        
        self.current_state = RobotState.IDLE
        self.target_data = None   
        self.target_room_name = ""
        self.rooms_dict = {}

        # ตัวแปรตามที่คุณกำหนด (เปลี่ยนจากตัวเลขเป็นข้อความ)
        self.mode = ['map', 'localize']
        self.way = ['go', 'back']
        self.default_way = self.way[0] # 'back'
        self.goal = [0.0, 0.0]        
        self.back_goal = [0.0, 0.0]   
        self.floor = 0
        self.update_floor = 0
        self.update_goal = [0, 0]
        
        self.service_called = False

        self.read_command()

        self.sub_room = self.create_subscription(String, '/room_target', self.room_callback, 10)

        self.cli = self.create_client(CheckFloor, 'check_floor_service')
        self.cli_open_map = self.create_client(OpenMap, 'open_map_service') # Client ตัวที่สอง
        self.cli_localize = self.create_client(CheckLocalize, 'check_localization_service')
        self.cli_pos = self.create_client(CheckPosition, 'check_position_service')
        self.cli_nav2 = self.create_client(Nav2, 'nav2_service')
        
        self.pub_final_target = self.create_publisher(Float32MultiArray, 'final_target', 10)


        self.timer = self.create_timer(1.0, self.state_machine_control)
        self.get_logger().info("🤖 State Manager Ready. Waiting for order...")

    def read_command(self):
        try:
            pkg_share = get_package_share_directory('my_voice_control')
            json_path = os.path.join(pkg_share, 'commands.json')
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.rooms_dict = data.get("rooms", {})
            self.get_logger().info(f"📚 Loaded {len(self.rooms_dict)} rooms.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to read JSON: {e}")
    
    def room_callback(self, msg):
        if self.current_state != RobotState.IDLE:
            self.get_logger().warn("⚠️ Robot is busy!")
            return

        room_key = msg.data
        if room_key in self.rooms_dict:
            self.target_room_name = room_key
            self.target_data = self.rooms_dict[room_key] 
            
            self.floor = float(self.target_data.get('floor', 0))
            
            # 2. ตรวจสอบเงื่อนไข default_way เพื่อเลือกพิกัดเป้าหมายเริ่มต้น
            if self.default_way == 'go':
                coords = self.target_data.get('go', [0.0, 0.0])
            else: # default_way == 'back'
                coords = self.target_data.get('back', [0.0, 0.0])
            
            self.goal = [float(coords[0]), float(coords[1])]

            self.get_logger().info(f"🔎 Target: {self.target_room_name} | Way: {self.default_way} | Goal: {self.goal}")
            
            # รีเซ็ตค่าสำหรับภารกิจใหม่
            self.update_goal = [0.0, 0.0]
            self.update_floor = self.floor # เบื้องต้นให้เท่ากับเป้าหมายก่อน
            self.service_called = False 
            self.current_state = RobotState.CHECK_FLOOR
        else:
            self.get_logger().error(f"❌ Room '{room_key}' not found.")

    def state_machine_control(self):
        if self.current_state == RobotState.IDLE:
            return

        self.get_logger().info(f"🔄 Current State: {self.current_state.name}")

        if self.current_state == RobotState.CHECK_FLOOR:
            if not self.service_called:
                self.call_check_floor_service()
                self.service_called = True
            else:
                self.get_logger().info("⏳ Waiting for floor service response...")

        elif self.current_state == RobotState.OPEN_MAP:
            if not self.service_called:
                self.call_open_map_service() # เรียก Service Open Map
                self.service_called = True

            # --- STATE: CHECK_LOCALIZE (แก้ไขใหม่) ---
        elif self.current_state == RobotState.CHECK_LOCALIZE:
            if not self.service_called:
                self.call_check_localize_service()
                self.service_called = True
            else:
                self.get_logger().info("⏳ Waiting for Localization to confirm...")

        elif self.current_state == RobotState.CHECK_POSITION:
            if not self.service_called:
                self.call_check_position_service()
                self.service_called = True

            # --- STATE: NAV2 ---
        # --- STATE: NAV2 (จุดหมายสุดท้าย) ---
        elif self.current_state == RobotState.NAV2:
                    if not self.service_called:
                        # ตรวจสอบพิกัด
                        if self.goal == [0.0, 0.0]:
                            self.get_logger().error("❌ Goal is [0, 0]. Moving to IDLE.")
                            self.current_state = RobotState.IDLE
                            return

                        # เรียกใช้ฟังก์ชันที่แยกไว้ (สะอาดกว่าและจัดการ error ง่ายกว่า)
                        self.call_nav2_service() 
                        self.service_called = True 
                    else:
                        self.get_logger().info("🚢 Robot is navigating... waiting for arrival.", throttle_duration_sec=5.0)

    def call_check_floor_service(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = CheckFloor.Request()
        request.floor = float(self.floor)
        
        self.get_logger().info(f"📡 Calling Service for floor: {self.floor}")
        future = self.cli.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"🏢 Current Floor: {response.current_floor} | Status: {response.status}")
            
            if response.status == "same_floor":
                self.get_logger().info("✅ Floor Match! Moving to next state.")
                self.update_goal = self.goal 
                self.update_floor = response.current_floor
                
                # --- จุดที่ต้องเพิ่ม ---
                self.service_called = False # ปลดล็อคเพื่อให้ State ถัดไปเรียก Service ได้
                self.current_state = RobotState.OPEN_MAP
            else:
                new_room_key = f"{response.status}_Stair{int(response.current_floor)}"
                self.get_logger().warn(f"🔄 Floor Mismatch! Switching target to: {new_room_key}")

                if new_room_key in self.rooms_dict:
                    stair_data = self.rooms_dict[new_room_key]
                    stair_coords = stair_data.get('go', [0.0, 0.0])
                    self.update_goal = [float(stair_coords[0]), float(stair_coords[1])]
                    self.update_floor = response.current_floor
                    
                    # --- จุดที่ต้องเพิ่ม ---
                    self.service_called = False # ปลดล็อคกรณีไปบันได
                    self.current_state = RobotState.OPEN_MAP
                else:
                    self.get_logger().error(f"❌ Target '{new_room_key}' not found.")
                    self.current_state = RobotState.IDLE
                    
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.current_state = RobotState.IDLE

    def call_open_map_service(self):
        while not self.cli_open_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for OpenMap service...')
        
        req = OpenMap.Request()
        # ส่งข้อมูลตามเงื่อนไขที่คุณระบุ
        req.mode = self.mode[1]       # 'localize'
        req.way = self.default_way       # 'back'
        req.floor = float(self.update_floor)
        
        self.get_logger().info(f"📡 Calling OpenMap: Mode={req.mode}, Way={req.way}, Floor={req.floor}")
        future = self.cli_open_map.call_async(req)
        future.add_done_callback(self.open_map_response_callback)

    def open_map_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"🗺️ Map Status: {response.status}")
            if response.status == "success": # สมมติว่าคืนค่า success เมื่อเปิดแผนที่เสร็จ
                self.service_called = False 
                self.current_state = RobotState.CHECK_LOCALIZE
            else:
                self.get_logger().error(f"❌ OpenMap Failed: {response.status}")
                self.current_state = RobotState.IDLE
        except Exception as e:
            self.get_logger().error(f"Service OpenMap failed: {e}")
            self.current_state = RobotState.IDLE

    def call_check_localize_service(self):
        while not self.cli_localize.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for CheckLocalize service...')
        
        req = CheckLocalize.Request()
        req.active = True  # ส่งค่า 1 (True) เพื่อ Activate
        
        self.get_logger().info("📡 Activating Localization Check...")
        future = self.cli_localize.call_async(req)
        future.add_done_callback(self.localize_response_callback)

    def localize_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("🎯 Localization Confirmed! Robot knows its position.")
                self.service_called = False # ปลดล็อค Flag
                self.current_state = RobotState.CHECK_POSITION
            else:
                self.get_logger().warn("⚠️ Localization failed. Retrying...")
                self.service_called = False # ให้มันเรียกซ้ำใน Loop ถัดไป หรือจัดการ Error ตามเหมาะสม
        except Exception as e:
            self.get_logger().error(f"Service CheckLocalize failed: {e}")
            self.current_state = RobotState.IDLE

    def call_check_position_service(self):
        while not self.cli_pos.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for CheckPosition service...')
        
        req = CheckPosition.Request()
        
        # เลือกพิกัดเป้าหมาย (ถ้ามีพิกัดบันไดที่เพิ่งอัปเดตมาให้ใช้ตัวนั้น)
        target = self.update_goal if self.update_goal != [0, 0] else self.goal
        
        req.x = float(target[0])
        req.y = float(target[1])
        req.way = self.default_way # ส่ง 'back' ตามที่ตั้งไว้
        
        self.get_logger().info(f"📡 Sending Position: X={req.x}, Y={req.y}, Way={req.way}")
        future = self.cli_pos.call_async(req)
        future.add_done_callback(self.position_response_callback)

    def position_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"✅ Position Confirmed. Updated Way: {response.update_way}")
            
            # กรณีมีการสลับทิศทาง (เช่น จาก back เป็น go)
            if self.default_way != response.update_way:
                self.get_logger().warn(f"🔄 Way changed to {response.update_way}. Re-fetching coordinates...")
                
                # 1. อัปเดตค่า Way หลักของระบบ
                self.default_way = response.update_way
                
                # 2. ย้อนกลับไป IDLE เพื่อเตรียมโหลดข้อมูลใหม่
                self.current_state = RobotState.IDLE
                self.service_called = False

                # 3. สั่งให้โหลดพิกัดใหม่ของห้องเดิม (target_room_name) ตาม Way ใหม่
                if self.target_room_name in self.rooms_dict:
                    self.get_logger().info(f"♻️ Re-loading {self.target_room_name} for {self.default_way} mode...")
                    
                    # ดึงข้อมูลพิกัดใหม่จาก JSON ตาม Way ที่เพิ่งได้มา
                    new_coords = self.target_data.get(self.default_way, [0.0, 0.0])
                    self.goal = [float(new_coords[0]), float(new_coords[1])]
                    
                    self.get_logger().info(f"📍 New Goal set to: {self.goal}")
                    
                    # 4. หลังจากโหลดพิกัดเสร็จ ให้เริ่มกระบวนการใหม่ทันที (Auto-start)
                    # หรือถ้าต้องการให้หยุดรอคำสั่งใหม่จริงๆ ก็ไม่ต้องเปลี่ยน State ตรงนี้ครับ
                    self.current_state = RobotState.CHECK_FLOOR 
                else:
                    self.get_logger().error("❌ Failed to re-load room data.")

            else:
                # กรณีทิศทางถูกต้องแล้ว (ตรงกัน)
                self.get_logger().info("🚀 Destination confirmed. Starting Navigation...")
                self.service_called = False
                self.current_state = RobotState.NAV2
                
        except Exception as e:
            self.get_logger().error(f"❌ Service CheckPosition failed: {e}")
            self.current_state = RobotState.IDLE

    def call_nav2_service(self):
            # --- เพิ่มการรอ Service ให้ชัวร์ขึ้น ---
            self.get_logger().info('📡 Checking nav2_service availability...')
            
            # รอสูงสุด 10 วินาที ถ้ายังไม่มาให้ยกเลิกก่อน
            if not self.cli_nav2.wait_for_service(timeout_sec=10.0):
                self.get_logger().error('❌ nav2_service is NOT ONLINE after 10s! Check your nav_goal node.')
                self.service_called = False # ให้ Timer รอบหน้าลองใหม่
                return

            req = Nav2.Request()
            req.x = float(self.goal[0]) 
            req.y = float(self.goal[1])
            
            self.get_logger().info(f"🛰️ Sending Final Goal to Nav2: X={req.x}, Y={req.y}")
            future = self.cli_nav2.call_async(req)
            future.add_done_callback(self.nav2_response_callback)

    def nav2_response_callback(self, future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("🏁 [SUCCESS] Robot reached the destination!")
                else:
                    self.get_logger().error("❌ [FAILED] Robot could not reach the destination.")
                
                # --- จบภารกิจ: รีเซ็ตทุกอย่างกลับไป IDLE ---
                self.get_logger().info("💤 System returning to IDLE state. Ready for new orders.")
                self.current_state = RobotState.IDLE
                self.service_called = False
                self.target_room_name = ""
                self.goal = [0.0, 0.0]
                self.update_goal = [0.0, 0.0]
                
            except Exception as e:
                self.get_logger().error(f"❌ Service Nav2 call failed: {e}")
                self.current_state = RobotState.IDLE
                self.service_called = False
def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    
    # ใช้ Executor แทนการ spin ปกติ
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()