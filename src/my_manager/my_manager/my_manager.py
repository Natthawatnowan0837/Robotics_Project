import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from enum import Enum
import json
import os
import subprocess
import signal
import time

from ament_index_python.packages import get_package_share_directory
from my_command.srv import CheckFloor
from my_command.srv import OpenMap
from my_command.srv import CheckLocalize
from my_command.srv import CheckPosition
from my_command.srv import Nav2
from my_command.srv import GotoStair
from my_command.srv import Controller

class RobotState(Enum):
    IDLE = 0
    SETUP = 1          
    CHECK_FLOOR = 2  
    OPEN_MAP = 3      
    CHECK_LOCALIZE = 4     
    CHECK_POSITION = 5   
    NAV2 = 6         
    GoToStair = 7
    Controller = 8

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        
        # --- Variables ---
        self.current_state = RobotState.IDLE
        self.target_data = None   
        self.target_room_name = ""
        self.rooms_dict = {}
        self.floor_status = "same_floor"
        
        self.mode = ['map', 'localize']
        self.way = ['go', 'back']
        self.default_way = self.way[1] # เริ่มต้นที่ 'back'
        
        self.goal = [0.0, 0.0]        
        self.floor = 0
        self.update_floor = 0
        self.update_goal = [0.0, 0.0]
        
        self.launch_process = None
        self.service_called = False
        self.setup_timer = None

        # --- Load Data ---
        self.read_command()

        # --- Publishers & Subscriptions ---
        self.sub_room = self.create_subscription(String, '/room_target', self.room_callback, 10)
        self.pub_current_state = self.create_publisher(String, '/robot_current_state', 10)
        
        # --- Service Clients ---
        self.cli_floor = self.create_client(CheckFloor, 'check_floor_service')
        self.cli_open_map = self.create_client(OpenMap, 'open_map_service')
        self.cli_localize = self.create_client(CheckLocalize, 'check_localization_service')
        self.cli_pos = self.create_client(CheckPosition, 'check_position_service')
        self.cli_nav2 = self.create_client(Nav2, 'nav2_service')
        self.cli_stair = self.create_client(GotoStair, 'goto_stair_service')
        self.cli_controller = self.create_client(Controller, 'controller_service')
        
        # --- Timers ---
        self.create_timer(0.5, self.publish_state)
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

    def publish_state(self):
        msg = String()
        msg.data = self.current_state.name
        self.pub_current_state.publish(msg)

    def room_callback(self, msg):
        if self.current_state != RobotState.IDLE:
            self.get_logger().warn("⚠️ Robot is busy!")
            return

        room_key = msg.data
        if room_key in self.rooms_dict:
            self.target_room_name = room_key
            self.target_data = self.rooms_dict[room_key] 
            self.floor = float(self.target_data.get('floor', 0))
            
            coords = self.target_data.get(self.default_way, [0.0, 0.0])
            self.goal = [float(coords[0]), float(coords[1])]

            self.get_logger().info(f"🔎 Target: {self.target_room_name} | Goal: {self.goal}")
            
            # เริ่มต้นภารกิจใหม่ที่ SETUP เสมอ
            self.update_goal = [0.0, 0.0]
            self.update_floor = self.floor
            self.service_called = False 
            self.current_state = RobotState.SETUP
        else:
            self.get_logger().error(f"❌ Room '{room_key}' not found.")

    def state_machine_control(self):
        if self.current_state == RobotState.IDLE:
            return

        self.get_logger().info(f"🔄 State: {self.current_state.name}", throttle_duration_sec=2.0)

        # --- STATE: SETUP (Launch ROS2) ---
        if self.current_state == RobotState.SETUP:
            if not self.service_called:
                self.get_logger().info("🛠️ Launching Fusion System...")
                self.terminate_launch_file() # Clean start
                if self.execute_fusion_launch():
                    self.service_called = True
                    # รอ 5 วินาทีให้ Sensor/Service พร้อม
                    self.setup_timer = self.create_timer(5.0, self.finish_setup_callback)
                else:
                    self.reset_to_idle()

        # --- STATE: CHECK_FLOOR ---
        elif self.current_state == RobotState.CHECK_FLOOR:
            if not self.service_called:
                self.call_service_async(self.cli_floor, CheckFloor.Request(floor=float(self.floor)), self.floor_response_callback)
                self.service_called = True

        # --- STATE: OPEN_MAP ---
        elif self.current_state == RobotState.OPEN_MAP:
            if not self.service_called:
                req = OpenMap.Request(mode=self.mode[1], way=self.default_way, floor=float(self.update_floor))
                self.call_service_async(self.cli_open_map, req, self.open_map_response_callback)
                self.service_called = True

        # --- STATE: CHECK_LOCALIZE ---
        elif self.current_state == RobotState.CHECK_LOCALIZE:
            if not self.service_called:
                self.call_service_async(self.cli_localize, CheckLocalize.Request(active=True), self.localize_response_callback)
                self.service_called = True

        # --- STATE: CHECK_POSITION ---
        elif self.current_state == RobotState.CHECK_POSITION:
            if not self.service_called:
                target = self.update_goal if self.update_goal != [0.0, 0.0] else self.goal
                req = CheckPosition.Request(x=float(target[0]), y=float(target[1]), way=self.default_way)
                self.call_service_async(self.cli_pos, req, self.position_response_callback)
                self.service_called = True

        # --- STATE: NAV2 ---
        elif self.current_state == RobotState.NAV2:
            if not self.service_called:
                target = self.update_goal if self.update_goal != [0.0, 0.0] else self.goal
                if target == [0.0, 0.0]:
                    self.reset_to_idle()
                    return
                self.call_service_async(self.cli_nav2, Nav2.Request(x=float(target[0]), y=float(target[1])), self.nav2_response_callback)
                self.service_called = True

        # --- STATE: GoToStair ---
        elif self.current_state == RobotState.GoToStair:
            if not self.service_called:
                self.call_service_async(self.cli_stair, GotoStair.Request(active=True), self.stair_response_callback)
                self.service_called = True

        # --- STATE: Controller ---
        elif self.current_state == RobotState.Controller:
            if not self.service_called:
                self.call_service_async(self.cli_controller, Controller.Request(active=True), self.controller_response_callback)
                self.service_called = True

    # --- Helper Functions ---
    def call_service_async(self, client, request, callback):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {client.srv_name}...')
        future = client.call_async(request)
        future.add_done_callback(callback)

    def finish_setup_callback(self):
        self.setup_timer.cancel()
        self.service_called = False
        self.current_state = RobotState.CHECK_FLOOR
        self.get_logger().info("✅ Fusion Ready. Moving to CHECK_FLOOR.")

    # --- Callbacks ---
    def floor_response_callback(self, future):
        try:
            res = future.result()
            self.floor_status = res.status
            if res.status == "same_floor":
                self.update_goal = self.goal
                self.update_floor = res.current_floor
                self.current_state = RobotState.OPEN_MAP
            else:
                new_key = f"{res.status}_Stair{int(res.current_floor)}"
                if new_key in self.rooms_dict:
                    coords = self.rooms_dict[new_key].get(self.default_way, [0.0, 0.0])
                    self.update_goal = [float(coords[0]), float(coords[1])]
                    self.update_floor = res.current_floor
                    self.current_state = RobotState.OPEN_MAP
                else:
                    self.reset_to_idle()
            self.service_called = False
        except Exception as e:
            self.get_logger().error(f"Floor Service Error: {e}")
            self.reset_to_idle()

    def open_map_response_callback(self, future):
        try:
            if future.result().status == "success":
                self.current_state = RobotState.CHECK_LOCALIZE
            else:
                self.reset_to_idle()
            self.service_called = False
        except: self.reset_to_idle()

    def localize_response_callback(self, future):
        try:
            if future.result().success:
                self.current_state = RobotState.CHECK_POSITION
            self.service_called = False
        except: self.reset_to_idle()

    def position_response_callback(self, future):
        try:
            res = future.result()
            if self.default_way != res.update_way:
                self.default_way = res.update_way
                new_coords = self.target_data.get(self.default_way, [0.0, 0.0])
                self.goal = [float(new_coords[0]), float(new_coords[1])]
                self.current_state = RobotState.CHECK_FLOOR # Restart sequence with new way
            else:
                self.current_state = RobotState.NAV2
            self.service_called = False
        except: self.reset_to_idle()

    def nav2_response_callback(self, future):
        try:
            if future.result().success:
                if self.floor_status in ["Up", "Down"]:
                    self.current_state = RobotState.GoToStair
                else:
                    self.reset_to_idle()
            else: self.reset_to_idle()
            self.service_called = False
        except: self.reset_to_idle()

    def stair_response_callback(self, future):
        try:
            if future.result().success:
                self.current_state = RobotState.Controller
            else: self.reset_to_idle()
            self.service_called = False
        except: self.reset_to_idle()

    def controller_response_callback(self, future):
        try:
            if future.result().success:
                self.current_state = RobotState.CHECK_FLOOR # Loop back to check floor
            else: self.reset_to_idle()
            self.service_called = False
        except: self.reset_to_idle()

    # --- Subprocess Management ---
    def execute_fusion_launch(self):
        cmd = "ros2 launch my_fusion fusion_launch.py"
        try:
            self.launch_process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
            return True
        except Exception as e:
            self.get_logger().error(f"Launch Error: {e}")
            return False

    def terminate_launch_file(self):
        if self.launch_process:
            try:
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.launch_process.wait(timeout=2)
            except: pass
            self.launch_process = None

    def reset_to_idle(self):
        self.get_logger().info("💤 Mission Finished or Error. Resetting to IDLE.")
        self.call_stop_map_service()
        self.terminate_launch_file()
        self.current_state = RobotState.IDLE
        self.service_called = False
        self.update_goal = [0.0, 0.0]

    def call_stop_map_service(self):
        if self.cli_open_map.wait_for_service(timeout_sec=0.5):
            self.cli_open_map.call_async(OpenMap.Request(mode="stop"))

def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.terminate_launch_file()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()