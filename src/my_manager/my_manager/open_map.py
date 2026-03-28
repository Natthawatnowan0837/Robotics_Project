#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import GetCostmap  # ใช้เช็คความพร้อมของ Nav2
import subprocess
import threading
import time
import os
import signal
from my_command.srv import OpenMap 
from ament_index_python.packages import get_package_share_directory 

class OpenMapServer(Node):
    def __init__(self):
        super().__init__('open_map_server')
        
        # เก็บ Process แยกกัน
        self.current_process = None # สำหรับ RTAB-Map
        self.nav2_process = None    # สำหรับ Nav2
        
        self.current_launch_id = "" 
        
        # สร้าง Service Client ภายใน Node เพื่อเช็คสถานะ Nav2
        # เราจะเช็คที่ /global_costmap/get_costmap เพราะถ้าตัวนี้มา แปลว่า Nav2 stack ส่วนใหญ่พร้อมแล้ว
        self.nav2_check_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')

        # สร้าง Service Server สำหรับรับคำสั่งเปิดแผนที่
        self.srv = self.create_service(
            OpenMap, 
            'open_map_service', 
            self.open_map_callback
        )
        
        self.get_logger().info("🤖 OpenMap Service Server Ready (Waiting for Nav2 check).")
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import GetCostmap  # ใช้เช็คความพร้อมของ Nav2
import subprocess
import threading
import time
import os
import signal
from my_command.srv import OpenMap 
from ament_index_python.packages import get_package_share_directory 

class OpenMapServer(Node):
    def __init__(self):
        super().__init__('open_map_server')
        
        # เก็บ Process แยกกัน
        self.current_process = None # สำหรับ RTAB-Map
        self.nav2_process = None    # สำหรับ Nav2
        
        self.current_launch_id = "" 
        
        # สร้าง Service Client ภายใน Node เพื่อเช็คสถานะ Nav2
        # เราจะเช็คที่ /global_costmap/get_costmap เพราะถ้าตัวนี้มา แปลว่า Nav2 stack ส่วนใหญ่พร้อมแล้ว
        self.nav2_check_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')

        # สร้าง Service Server สำหรับรับคำสั่งเปิดแผนที่
        self.srv = self.create_service(
            OpenMap, 
            'open_map_service', 
            self.open_map_callback
        )
        
        self.get_logger().info("🤖 OpenMap Service Server Ready (Waiting for Nav2 check).")

    def wait_for_nav2_ready(self, timeout=30.0):
        """
        ฟังก์ชันวนลูปเช็คจนกว่า Service ของ Nav2 จะปรากฏ
        """
        self.get_logger().info("⏳ Waiting for Nav2 servers to initialize...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            # เช็คว่า Service มาหรือยัง (รอครั้งละ 1 วินาที)
            if self.nav2_check_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("✨ Nav2 is fully active and costmap is ready!")
                return True
            
            # เช็คว่า Process Nav2 ยังรันอยู่ไหม (เผื่อมัน Crash ระหว่างเริ่ม)
            if self.nav2_process and self.nav2_process.poll() is not None:
                self.get_logger().error("💀 Nav2 process terminated unexpectedly during startup.")
                return False
                
            self.get_logger().info("... still waiting for Nav2 ...")
            
        return False

    def open_map_callback(self, request, response):
        mode_val = request.mode 
        way_val = request.way   
        floor_num = request.floor
        
        floor_str = f"floor{int(floor_num)}"
        launch_id = f"{mode_val}_{way_val}_{floor_str}"

        if self.current_launch_id == launch_id:
            self.get_logger().info(f"ℹ️ {launch_id} is already running.")
            response.status = "success"
            return response

        self.get_logger().info(f"📥 Service Request: Mode={mode_val}, Floor={floor_str}, Way={way_val}")
        
        # รันระบบ Launch
        success = self.run_launch(mode_val, floor_str, way_val)
        
        if success:
            self.current_launch_id = launch_id
            response.status = "success"
            self.get_logger().info(f"✅ Service Response sent: SUCCESS")
        else:
            response.status = "failed"
            self.get_logger().error(f"❌ Service Response sent: FAILED")
            
        return response

    def run_launch(self, mode_name, floor, direction):
        # 1. ปิด Process เก่า
        for proc in [self.current_process, self.nav2_process]:
            if proc:
                try:
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    time.sleep(1.0)
                    os.killpg(pgid, signal.SIGKILL)
                except:
                    pass
        
        self.current_process = None
        self.nav2_process = None

        # 2. Cleanup GUI
        subprocess.run(['pkill', '-9', 'rtabmap_viz'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', 'rviz2'], stderr=subprocess.DEVNULL)
        time.sleep(0.5)

        # 3. เริ่มรัน RTAB-Map
        launch_file = 'launch_mapping.launch.py' if mode_name == 'map' else 'launch_localize.launch.py'
        rtab_command = [
            'ros2', 'launch', 'my_manager', launch_file,
            f'floor:={floor}',
            f'db_name:={direction}'
        ]
        
        try:
            self.get_logger().info(f"🚀 Launching RTAB: {mode_name.upper()}")
            self.current_process = subprocess.Popen(rtab_command, preexec_fn=os.setsid)
            
            # รอ RTAB ตั้งตัวสักครู่ (ปกติ RTAB จะเร็วกว่า Nav2)
            time.sleep(10.0) 

            # 4. เริ่มรัน Nav2
            try:
                pkg_path = get_package_share_directory('my_manager')
                nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
            except:
                nav2_params = '/home/noone/Robotics_Project/src/my_manager/config/nav2_params.yaml'

            nav2_command = [
                'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                'use_sim_time:=false',
                f'params_file:={nav2_params}',
                'use_amcl:=false', # ใช้ RTAB แทน AMCL
                'map:=/rtabmap/map'
            ]

            self.get_logger().info("🚀 Launching Nav2 Bringup...")
            self.nav2_process = subprocess.Popen(nav2_command, preexec_fn=os.setsid)
            
            # --- ส่วนสำคัญ: รอจนกว่า Nav2 จะพร้อมจริงๆ ก่อนส่ง Success ---
            nav2_is_ready = self.wait_for_nav2_ready(timeout=40.0)
            
            if nav2_is_ready:
                return True
            else:
                self.get_logger().error("❌ Nav2 failed to initialize in time.")
                return False

        except Exception as e:
            self.get_logger().error(f"❌ Launch System Error: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = OpenMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # ปิดทุกอย่างเมื่อโดน Ctrl+C
        for proc in [node.current_process, node.nav2_process]:
            if proc:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except:
                    pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    def wait_for_nav2_ready(self, timeout=30.0):
        """
        ฟังก์ชันวนลูปเช็คจนกว่า Service ของ Nav2 จะปรากฏ
        """
        self.get_logger().info("⏳ Waiting for Nav2 servers to initialize...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            # เช็คว่า Service มาหรือยัง (รอครั้งละ 1 วินาที)
            if self.nav2_check_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("✨ Nav2 is fully active and costmap is ready!")
                return True
            
            # เช็คว่า Process Nav2 ยังรันอยู่ไหม (เผื่อมัน Crash ระหว่างเริ่ม)
            if self.nav2_process and self.nav2_process.poll() is not None:
                self.get_logger().error("💀 Nav2 process terminated unexpectedly during startup.")
                return False
                
            self.get_logger().info("... still waiting for Nav2 ...")
            
        return False

    def open_map_callback(self, request, response):
        mode_val = request.mode 
        way_val = request.way   
        floor_num = request.floor
        
        floor_str = f"floor{int(floor_num)}"
        launch_id = f"{mode_val}_{way_val}_{floor_str}"

        if self.current_launch_id == launch_id:
            self.get_logger().info(f"ℹ️ {launch_id} is already running.")
            response.status = "success"
            return response

        self.get_logger().info(f"📥 Service Request: Mode={mode_val}, Floor={floor_str}, Way={way_val}")
        
        # รันระบบ Launch
        success = self.run_launch(mode_val, floor_str, way_val)
        
        if success:
            self.current_launch_id = launch_id
            response.status = "success"
            self.get_logger().info(f"✅ Service Response sent: SUCCESS")
        else:
            response.status = "failed"
            self.get_logger().error(f"❌ Service Response sent: FAILED")
            
        return response

    def run_launch(self, mode_name, floor, direction):
            # 1. ปิด Process เก่า (เหมือนเดิม)
            for proc in [self.current_process, self.nav2_process]:
                if proc:
                    try:
                        pgid = os.getpgid(proc.pid)
                        os.killpg(pgid, signal.SIGTERM)
                        time.sleep(1.0)
                        os.killpg(pgid, signal.SIGKILL)
                    except:
                        pass
            
            self.current_process = None
            self.nav2_process = None

            # 2. Cleanup GUI
            subprocess.run(['pkill', '-9', 'rtabmap_viz'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', 'rviz2'], stderr=subprocess.DEVNULL)
            time.sleep(0.5)

            # 3. เริ่มรัน RTAB-Map
            launch_file = 'launch_mapping.launch.py' if mode_name == 'map' else 'launch_localize.launch.py'
            rtab_command = [
                'ros2', 'launch', 'my_manager', launch_file,
                f'floor:={floor}',
                f'db_name:={direction}'
            ]
            
            try:
                self.get_logger().info(f"🚀 Launching RTAB: {mode_name.upper()}")
                self.current_process = subprocess.Popen(rtab_command, preexec_fn=os.setsid)
                
                # รอ RTAB ตั้งตัวสักครู่
                time.sleep(10.0) 

                # 4. เริ่มรัน Nav2 พร้อม Remapping สำหรับ Twist Mux
                try:
                    # ดึง Path จาก Package my_manager
                    pkg_path = get_package_share_directory('my_manager')
                    nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
                except Exception:
                    # Fallback path กรณีหา package ไม่เจอ
                    nav2_params = '/home/noone/Robotics_Project/src/my_manager/config/nav2_params.yaml'

                nav2_command = [
                    'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                    'use_sim_time:=false',
                    f'params_file:={nav2_params}',
                    'use_amcl:=false', # ใช้ RTAB แทน AMCL
                    'map:=/rtabmap/map',
                    # --- [เพิ่มการ Remap ตรงนี้] ---
                    '--ros-args', 
                    '-r', '/cmd_vel:=/cmd_vel_nav2'
                ]

                self.get_logger().info(f"🚀 Launching Nav2 with Mux Remapping: /cmd_vel -> /cmd_vel_nav2")
                self.nav2_process = subprocess.Popen(nav2_command, preexec_fn=os.setsid)
                
                # รอจนกว่า Nav2 จะพร้อมจริงๆ
                nav2_is_ready = self.wait_for_nav2_ready(timeout=40.0)
                
                if nav2_is_ready:
                    return True
                else:
                    self.get_logger().error("❌ Nav2 failed to initialize in time.")
                    return False

            except Exception as e:
                self.get_logger().error(f"❌ Launch System Error: {e}")
                return False

def main(args=None):
    rclpy.init(args=args)
    node = OpenMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # ปิดทุกอย่างเมื่อโดน Ctrl+C
        for proc in [node.current_process, node.nav2_process]:
            if proc:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except:
                    pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()