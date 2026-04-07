#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.srv import GetCostmap  
import subprocess
import os
import signal
import time
from my_command.srv import OpenMap 
from ament_index_python.packages import get_package_share_directory 

class OpenMapServer(Node):
    def __init__(self):
        super().__init__('open_map_server')
        
        # เก็บ Process แยกกันเพื่อการจัดการที่แม่นยำ
        self.current_process = None # สำหรับ RTAB-Map
        self.nav2_process = None    # สำหรับ Nav2
        self.current_launch_id = "" 
        
        # Client สำหรับเช็คสถานะความพร้อมของ Nav2
        self.nav2_check_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')

        # สร้าง Service Server
        self.srv = self.create_service(
            OpenMap, 
            'open_map_service', 
            self.open_map_callback
        )
        
        self.get_logger().info("🤖 OpenMap Service Server Ready. Listening for Start/Stop commands...")

    def wait_for_nav2_ready(self, timeout=40.0):
        """ วนลูปเช็คจนกว่า Nav2 Costmap Service จะปรากฏ (แปลว่าระบบพร้อมเดิน) """
        self.get_logger().info("⏳ Waiting for Nav2 servers to initialize...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            if self.nav2_check_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("✨ Nav2 is fully active and costmap is ready!")
                return True
            
            if self.nav2_process and self.nav2_process.poll() is not None:
                self.get_logger().error("💀 Nav2 process terminated unexpectedly.")
                return False
                
            self.get_logger().info("... still waiting for Nav2 ...")
        return False

    def stop_all_processes(self):
        """ ฟังก์ชันสำหรับล้างระบบ Kill ทุกอย่างที่เปิดไว้ """
        self.get_logger().info("🧹 Cleaning up all navigation processes...")
        
        for proc in [self.current_process, self.nav2_process]:
            if proc:
                try:
                    # ใช้ Process Group ID (PGID) เพื่อ Kill ลูกหลานทั้งหมดของ Launch file
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    time.sleep(1.0)
                    os.killpg(pgid, signal.SIGKILL)
                except Exception as e:
                    self.get_logger().warn(f"Process cleanup notice: {e}")
        
        self.current_process = None
        self.nav2_process = None
        self.current_launch_id = ""

        # ล้างหน้าต่าง GUI ที่อาจค้างอยู่ (Force Kill)
        subprocess.run(['pkill', '-9', 'rtabmap_viz'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', 'rviz2'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', 'rtabmap'], stderr=subprocess.DEVNULL)
        self.get_logger().info("✅ All processes stopped. System is now IDLE.")

    def open_map_callback(self, request, response):
        mode_val = request.mode.lower() 
        way_val = request.way   
        floor_num = request.floor
        
        # --- [CASE: สั่งปิดระบบเมื่อ IDLE] ---
        if mode_val in ["idle", "stop", "none"]:
            self.stop_all_processes()
            response.status = "success"
            return response

        # --- [CASE: สั่งเปิดระบบ (Mapping/Localization)] ---
        floor_str = f"floor{int(floor_num)}"
        launch_id = f"{mode_val}_{way_val}_{floor_str}"

        # ถ้าตัวที่สั่งรันอยู่แล้ว ไม่ต้องทำอะไร
        if self.current_launch_id == launch_id:
            self.get_logger().info(f"ℹ️ {launch_id} is already running.")
            response.status = "success"
            return response

        self.get_logger().info(f"📥 New Request: Mode={mode_val}, Floor={floor_str}, Way={way_val}")
        
        # สั่งรันระบบใหม่
        success = self.run_launch(mode_val, floor_str, way_val)
        
        if success:
            self.current_launch_id = launch_id
            response.status = "success"
        else:
            response.status = "failed"
            
        return response

    def run_launch(self, mode_name, floor, direction):
        # 1. ปิดของเก่าก่อนทุกครั้งที่เริ่มรันใหม่
        self.stop_all_processes()
        time.sleep(1.0)

        # 2. เริ่มรัน RTAB-Map
        launch_file = 'launch_mapping.launch.py' if mode_name == 'map' else 'launch_localize.launch.py'
        rtab_command = [
            'ros2', 'launch', 'my_manager', launch_file,
            f'floor:={floor}',
            f'db_name:={direction}'
        ]
        
        try:
            self.get_logger().info(f"🚀 Launching RTAB: {mode_name.upper()}")
            # ใช้ preexec_fn=os.setsid เพื่อให้สามารถ Kill แบบยกกลุ่มได้ในภายหลัง
            self.current_process = subprocess.Popen(rtab_command, preexec_fn=os.setsid)
            
            # รอ RTAB ตั้งตัว
            time.sleep(8.0) 

            # 3. เริ่มรัน Nav2
            try:
                pkg_path = get_package_share_directory('my_manager')
                nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
            except:
                nav2_params = '/home/noone/Robotics_Project/src/my_manager/config/nav2_params.yaml'

            nav2_command = [
                'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                'use_sim_time:=false',
                f'params_file:={nav2_params}',
                'use_amcl:=false', 
                'map:=/rtabmap/map'
            ]

            self.get_logger().info("🚀 Launching Nav2 Bringup...")
            self.nav2_process = subprocess.Popen(nav2_command, preexec_fn=os.setsid)
            
            # รอจนกว่า Nav2 จะพร้อมใช้งานจริง
            return self.wait_for_nav2_ready(timeout=45.0)

        except Exception as e:
            self.get_logger().error(f"❌ Launch System Error: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = OpenMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_all_processes()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()