import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import json
import os
import subprocess
import signal
import time

from ament_index_python.packages import get_package_share_directory
from my_command.srv import CheckFloor, OpenMap, CheckLocalize, CheckPosition, Goal, Nav2, GotoStair, Controller

class RobotState(Enum):
    IDLE = 0
    SETUP = 1          
    CHECK_FLOOR = 2  
    OPEN_MAP = 3      
    CHECK_LOCALIZE = 4     
    CHECK_POSITION = 5  
    NAV2 = 6
    GOAL = 7 # แก้ไขจาก Goal เป็น GOAL
    GoToStair = 8
    Controller = 9

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        
        # --- Variables ---
        self.current_state = RobotState.IDLE
        self.target_data = None   
        self.rooms_dict = {}
        self.floor_status = "same_floor"
        self.mode = ['map', 'localize']
        self.way = ['go', 'back']
        self.default_way = self.way[1] 
        
        self.goal_coords = [0.0, 0.0] # เปลี่ยนชื่อตัวแปรเล็กน้อยเพื่อไม่ให้ซ้ำกับชื่อ State       
        self.floor = 0
        self.update_floor = 0
        self.update_goal = [0.0, 0.0]
        
        self.launch_process = None
        self.service_called = False
        self.setup_timer = None

        self.read_command()

        # --- Publishers & Subscriptions ---
        self.sub_room = self.create_subscription(String, '/room_target', self.room_callback, 10)
        self.pub_process = self.create_publisher(String, '/process', 10)
        # --- Service Clients ---
        self.cli_floor = self.create_client(CheckFloor, 'check_floor_service')
        self.cli_open_map = self.create_client(OpenMap, 'open_map_service')
        self.cli_localize = self.create_client(CheckLocalize, 'check_localization_service')
        self.cli_pos = self.create_client(CheckPosition, 'check_position_service')
        self.cli_nav2 = self.create_client(Nav2, 'Nav2_service')
        self.cli_goal = self.create_client(Goal, 'Goal_service')
        self.cli_stair = self.create_client(GotoStair, 'goto_stair_service')
        self.cli_controller = self.create_client(Controller, 'controller_service')

        self.create_timer(0.5, self.publish_all_status)
        self.timer = self.create_timer(1.0, self.state_machine_control)
        
        self.get_logger().info("🤖 State Manager Ready. State 'Goal' updated.")

    def publish_all_status(self):
        # สร้าง Message สำหรับ Topic 'process'
        process_msg = String()
        process_msg.data = f"Current Process: {self.current_state.name}"
        self.pub_process.publish(process_msg)

    # --- Subprocess Management ---
    def execute_fusion_launch(self):
        cmd = "ros2 launch my_fusion fusion_launch.py "
        try:
            self.get_logger().info(f"🚀 Launching Fusion: {cmd}")
            self.launch_process = subprocess.Popen(
                cmd, shell=True, preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Launch Error: {e}")
            return False

    def terminate_launch_file(self):
        if self.launch_process:
            try:
                # ส่งสัญญาณ Terminate ไปยังกลุ่ม Process ของ Fusion (RTAB-Map/EKF)
                pgid = os.getpgid(self.launch_process.pid)
                os.killpg(pgid, signal.SIGTERM)
                self.launch_process.wait(timeout=2)
            except: pass
            self.launch_process = None

        # 🧹 ล้างเฉพาะ RTAB-Map และ Nav2 แต่ "ไม่ล้าง" Realsense
        os.system("pkill -9 -f rtabmap")
        os.system("pkill -9 -f nav2") 
        # os.system("pkill -9 -f micro_ros_agent") # ถ้าหุ่นเดินอยู่ ห้ามฆ่า Agent ครับ ล้อจะค้าง!
        self.get_logger().info("🧹 Fusion Cleared. Camera is still alive.")

    # --- Data & Callback ---
    def read_command(self):
        try:
            pkg_share = get_package_share_directory('my_voice_control')
            json_path = os.path.join(pkg_share, 'commands.json')
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.rooms_dict = data.get("rooms", {})
        except Exception as e:
            self.get_logger().error(f"❌ JSON Error: {e}")

    def publish_state(self):
        msg = String()
        msg.data = self.current_state.name
        self.pub_current_state.publish(msg)

    def room_callback(self, msg):
        if self.current_state != RobotState.IDLE: return
        room_key = msg.data
        if room_key in self.rooms_dict:
            self.target_data = self.rooms_dict[room_key] 
            self.floor = float(self.target_data.get('floor', 0))
            coords = self.target_data.get(self.default_way, [0.0, 0.0])
            self.goal_coords = [float(coords[0]), float(coords[1])]
            self.update_goal = [0.0, 0.0]; self.update_floor = self.floor
            self.service_called = False 
            self.current_state = RobotState.SETUP

    # --- STATE MACHINE CONTROL ---
    def state_machine_control(self):
        if self.current_state == RobotState.IDLE: return
        self.get_logger().info(f"🔄 State: {self.current_state.name}", throttle_duration_sec=2.0)

        if self.current_state == RobotState.SETUP:
            if not self.service_called:
                # เอา self.terminate_launch_file() ออกตามที่ต้องการ
                # เพื่อให้ State นี้ทำหน้าที่แค่ "เริ่ม" (Launch) เท่านั้น
                if self.execute_fusion_launch():
                    self.service_called = True
                    # สร้าง Timer เพื่อรอให้ระบบ Fusion (RTAB-Map/EKF) ตั้งตัวได้ 5 วินาที
                    self.setup_timer = self.create_timer(5.0, self.finish_setup_callback)
                else: 
                    self.get_logger().error("❌ Failed to execute fusion launch")
                    self.reset_to_idle()

        elif self.current_state == RobotState.CHECK_FLOOR:
            if not self.service_called:
                self.call_service_async(self.cli_floor, CheckFloor.Request(floor=float(self.floor)), self.floor_response_callback)
                self.service_called = True

        elif self.current_state == RobotState.OPEN_MAP:
            if not self.service_called:
                req = OpenMap.Request(mode=self.mode[1], way=self.default_way, floor=float(self.update_floor))
                self.call_service_async(self.cli_open_map, req, self.open_map_response_callback)
                self.service_called = True

        elif self.current_state == RobotState.CHECK_LOCALIZE:
            if not self.service_called:
                self.call_service_async(self.cli_localize, CheckLocalize.Request(active=True), self.localize_response_callback)
                self.service_called = True

        elif self.current_state == RobotState.CHECK_POSITION:
            if not self.service_called:
                target = self.update_goal if self.update_goal != [0.0, 0.0] else self.goal_coords
                req = CheckPosition.Request(x=float(target[0]), y=float(target[1]), way=self.default_way)
                self.call_service_async(self.cli_pos, req, self.position_response_callback)
                self.service_called = True

        elif self.current_state == RobotState.NAV2:
            if not self.service_called:
                self.call_service_async(self.cli_nav2, Nav2.Request(active=True), self.nav2_response_callback)
                self.service_called = True

        elif self.current_state == RobotState.GOAL:
            if not self.service_called:
                # 1. รอจนกว่า Service จะปรากฏในระบบ ROS
                if not self.cli_goal.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info("⏳ Waiting for Goal Service to connect...")
                    return 

                # 2. เตรียมข้อมูลพิกัด
                target = self.update_goal if self.update_goal != [0.0, 0.0] else self.goal_coords
                req = Goal.Request()
                req.x = float(target[0])
                req.y = float(target[1])

                # 3. ส่ง Request และรอ Callback
                self.get_logger().info(f"🎯 System Ready! Sending Goal X:{req.x} Y:{req.y}")
                self.call_service_async(self.cli_goal, req, self.goal_response_callback)
                self.service_called = True

        elif self.current_state == RobotState.GoToStair:
            if not self.service_called:
                self.call_service_async(self.cli_stair, GotoStair.Request(active=True), self.stair_response_callback)
                self.service_called = True

        elif self.current_state == RobotState.Controller:
            if not self.service_called:
                self.call_service_async(self.cli_controller, Controller.Request(active=True), self.controller_response_callback)
                self.service_called = True

    # --- ALL CALLBACKS ---
    def call_service_async(self, client, request, callback):
        if not client.wait_for_service(timeout_sec=1.0): return
        future = client.call_async(request)
        future.add_done_callback(callback)

    def finish_setup_callback(self):
        self.setup_timer.cancel()
        self.service_called = False
        self.current_state = RobotState.CHECK_FLOOR

    def floor_response_callback(self, future):
        try:
            res = future.result()
            self.floor_status = res.status
            if res.status == "same_floor": self.update_goal = self.goal_coords
            else:
                new_key = f"{res.status}_Stair{int(res.current_floor)}"
                coords = self.rooms_dict.get(new_key, {}).get(self.default_way, [0.0, 0.0])
                self.update_goal = [float(coords[0]), float(coords[1])]
            self.update_floor = res.current_floor
            self.current_state = RobotState.OPEN_MAP
            self.service_called = False
        except: self.reset_to_idle()

    def open_map_response_callback(self, future):
        try:
            res = future.result()
            if res.status == "success":
                self.get_logger().info("🗺️ Map Service Started! Waiting 7s for stability...")
                
                # เปลี่ยนเป็น IDLE ชั่วคราวเพื่อหยุด State Machine ไม่ให้รันต่อ
                self.current_state = RobotState.IDLE 
                
                # สร้าง Timer เพื่อรอให้ระบบ Sensor และ Map พร้อมจริงๆ
                if hasattr(self, 'wait_timer') and self.wait_timer:
                    self.wait_timer.cancel()
                self.wait_timer = self.create_timer(7.0, self.confirm_ready_to_localize)
            else:
                self.get_logger().error("❌ OpenMap Failed")
                self.reset_to_idle()
            self.service_called = False
        except Exception as e:
            self.get_logger().error(f"❌ Error in OpenMap Callback: {e}")
            self.reset_to_idle()

    def confirm_ready_to_localize(self):
        """ ฟังก์ชันนี้จะถูกเรียกหลังผ่านไป 7 วินาที """
        self.wait_timer.cancel()
        self.get_logger().info("🚀 Systems Stable. Now starting CHECK_LOCALIZE...")
        self.current_state = RobotState.CHECK_LOCALIZE
        self.service_called = False # ปลดล็อกเพื่อให้เข้าสู่ State ถัดไป

    def wait_for_rtabmap_ready(self):
        self.timer_map.cancel() # หยุดตัวนับเวลา
        self.get_logger().info("🚀 RTAB-Map should be ready now. Starting Localize...")
        self.current_state = RobotState.CHECK_LOCALIZE # ค่อยเริ่ม Localize ตรงนี้
        self.service_called = False

    def localize_response_callback(self, future):
        try:
            if future.result().success: self.current_state = RobotState.CHECK_POSITION
            self.service_called = False
        except: self.reset_to_idle()

    def position_response_callback(self, future):
            try:
                res = future.result()
                if self.default_way != res.update_way:
                    # --- [ เพิ่มตรงนี้ ] ---
                    # สั่งหยุดระบบ Localize เดิมก่อนจะไป SETUP ใหม่
                    stop_loc_req = CheckLocalize.Request()
                    stop_loc_req.active = False
                    self.cli_localize.call_async(stop_loc_req) 
                    self.get_logger().info("🛑 Deactivating Localize before Setup...")
                    # ----------------------

                    self.default_way = res.update_way
                    new_coords = self.target_data.get(self.default_way, [0.0, 0.0])
                    self.goal_coords = [float(new_coords[0]), float(new_coords[1])]
                    
                    self.current_state = RobotState.SETUP 
                else: 
                    self.current_state = RobotState.NAV2
                
                self.service_called = False
            except Exception as e:
                self.get_logger().error(f"Error in position callback: {e}")
                self.reset_to_idle()

    def nav2_response_callback(self, future):
        try:
            if future.result().success: self.current_state = RobotState.GOAL
            self.service_called = False
        except: self.reset_to_idle()

    def goal_response_callback(self, future):
            try:
                response = future.result()
                
                # เช็ค status ก่อน (ระบบพร้อมรับงาน)
                if response.status:
                    self.get_logger().info("✅ Nav2 accepted the goal and is planning...")

                # เช็ค success (หุ่นยนต์เดินถึงที่หมาย)
                if response.success:
                    self.get_logger().info("🏁 Destination Reached! Checking next step...")
                    
                    # เงื่อนไข: ถ้า floor_status เป็น "Up" ให้ไปที่ GoToStair
                    if self.floor_status == "Up":
                        self.get_logger().info("🪜 Floor status is 'Up'. Moving to GoToStair.")
                        self.current_state = RobotState.GoToStair
                    else:
                        # ถ้าเป็น same_floor หรือ Down หรืออื่นๆ ให้จบภารกิจ
                        self.get_logger().info(f"✅ Mission Finished (Status: {self.floor_status}). Resetting to IDLE.")
                        self.current_state = RobotState.IDLE
                
                # ปลดล็อกเพื่อให้ State Machine ทำงานในรอบถัดไปได้
                    self.service_called = False
                else:
                    self.get_logger().error("❌ Navigation Failed.")
                    self.reset_to_idle()

            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                self.reset_to_idle()

    def stair_response_callback(self, future):
        try:
            self.current_state = RobotState.Controller if future.result().success else RobotState.IDLE
            if self.current_state == RobotState.IDLE: self.reset_to_idle()
            self.service_called = False
        except: self.reset_to_idle()

    def controller_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("✅ Climbing finished. Deactivating Localize and Resetting Setup...")
                
                # 1. สั่งหยุดระบบ Localize ทันที (ส่ง active: False)
                stop_loc_req = CheckLocalize.Request()
                stop_loc_req.active = False
                self.cli_localize.call_async(stop_loc_req)
                
                # 2. เปลี่ยนสถานะกลับไป SETUP เพื่อเริ่มกระบวนการของชั้นใหม่
                self.current_state = RobotState.SETUP
                
            else:
                self.get_logger().error("❌ Controller failed.")
                self.reset_to_idle()
                
            self.service_called = False
        except Exception as e:
            self.get_logger().error(f"❌ Exception in controller callback: {e}")
            self.reset_to_idle()

    def reset_to_idle(self):
        self.get_logger().info("💤 Mission Finished or Error. Resetting.")
        if self.cli_open_map.wait_for_service(timeout_sec=0.1):
            self.cli_open_map.call_async(OpenMap.Request(mode="stop"))
        self.terminate_launch_file()
        self.current_state = RobotState.IDLE
        self.service_called = False

def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.terminate_launch_file()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()