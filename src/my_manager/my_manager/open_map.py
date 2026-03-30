#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import signal
from rclpy.executors import MultiThreadedExecutor

# ROS2 Message & Service Imports
from my_command.srv import OpenMap 
from ament_index_python.packages import get_package_share_directory 

class OpenMapServer(Node):
    def __init__(self):
        super().__init__('open_map_server')
        
        # เก็บเฉพาะ Process ของ RTAB-Map
        self.current_process = None
        self.current_launch_id = "" 

        # สร้าง Service Server สำหรับรับคำสั่ง
        self.srv = self.create_service(
            OpenMap, 
            'open_map_service', 
            self.open_map_callback
        )
        
        self.get_logger().info("🤖 RTAB-Map Only Server (Multi-Threaded) Ready.")

    def cleanup_all(self):
        """ ล้าง Process RTAB-Map ให้เกลี้ยง แต่ไม่ฆ่า Hardware หลัก """
        self.get_logger().info("🧹 Cleaning up RTAB-Map processes...")
        
        if self.current_process:
            try:
                # ฆ่าเฉพาะกลุ่ม process ที่เราสั่ง launch
                pgid = os.getpgid(self.current_process.pid)
                os.killpg(pgid, signal.SIGKILL)
                self.current_process.wait(timeout=2.0)
            except:
                pass

        # ฆ่าเฉพาะ RTAB-Map และ GUI (ปล่อย realsense ไว้ ถ้ามันรันจากที่อื่น)
        targets = ['rtabmap', 'rtabmap_viz'] 
        for target in targets:
            subprocess.run(['pkill', '-9', '-f', target], stderr=subprocess.DEVNULL)
            
        self.current_process = None
        self.current_launch_id = ""
        # สำคัญมาก: รอให้ Database file คืน lock และ Port สื่อสารว่างลง
        time.sleep(2.0)

    def open_map_callback(self, request, response):
        mode_val = request.mode 
        way_val = request.way   
        floor_num = request.floor
        
        if mode_val == "stop":
            self.cleanup_all()
            response.status = "success"
            return response

        launch_id = f"{mode_val}_{way_val}_floor{int(floor_num)}"

        if self.current_launch_id == launch_id:
            self.get_logger().info(f"ℹ️ {launch_id} is already running.")
            response.status = "success"
            return response

        self.get_logger().info(f"📥 Request received: {launch_id}")
        
        # รันเฉพาะ RTAB-Map
        if self.run_rtab_launch(mode_val, int(floor_num), way_val):
            self.current_launch_id = launch_id
            response.status = "success"
        else:
            self.cleanup_all()
            response.status = "failed"
            
        return response

    def run_rtab_launch(self, mode_name, floor, direction):
        self.cleanup_all()

        floor_str = f"floor{floor}"
        # เลือกไฟล์ launch ให้ถูกโหมด
        launch_file = 'launch_mapping.launch.py' if mode_name == 'map' else 'launch_localize.launch.py'
        
        # --- เพิ่มส่วนนี้เพื่อ Debug Path ---
        home = os.path.expanduser('~')
        db_path = os.path.join(home, 'Robotics_Project/src/my_manager/maps', floor_str, f"{direction}.db")
        
        if mode_name == 'localize' and not os.path.exists(db_path):
            self.get_logger().error(f"❌ หาไฟล์ Database ไม่เจอที่: {db_path}")
            # ถ้าหาไม่เจอ แผนที่ก็จะไม่ขึ้น
        else:
            self.get_logger().info(f"📂 กำลังใช้ Database: {db_path}")
        # ---------------------------------

        try:
            rtab_cmd = [
                'ros2', 'launch', 'my_manager', launch_file,
                f'floor:={floor_str}', 
                f'db_name:={direction}'
            ]
            self.get_logger().info(f"🚀 Executing: {' '.join(rtab_cmd)}")
            self.current_process = subprocess.Popen(rtab_cmd, preexec_fn=os.setsid)
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Launch Error: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = OpenMapServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down...")
    finally:
        node.cleanup_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()