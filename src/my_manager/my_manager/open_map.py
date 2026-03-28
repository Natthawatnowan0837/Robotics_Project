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
        """ ล้าง Process RTAB-Map และ GUI ที่เกี่ยวข้องให้เกลี้ยง """
        self.get_logger().info("🧹 Cleaning up RTAB-Map processes...")
        
        # 1. Kill process ที่เก็บในตัวแปร
        if self.current_process and self.current_process.poll() is None:
            try:
                pgid = os.getpgid(self.current_process.pid)
                self.get_logger().info(f"🛑 Killing PGID: {pgid}")
                os.killpg(pgid, signal.SIGKILL)
                self.current_process.wait(timeout=1.0)
            except:
                pass

        # 2. กวาดล้างซากที่อาจหลงเหลือ (rtabmap, viz)
        targets = ['rtabmap', 'rtabmap_viz', 'realsense2_camera']
        for target in targets:
            subprocess.run(['pkill', '-9', '-f', target], stderr=subprocess.DEVNULL)
            
        self.current_process = None
        self.current_launch_id = ""
        time.sleep(1.0) # รอให้ Hardware/Port ว่าง

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
        launch_file = 'launch_mapping.launch.py' if mode_name == 'map' else 'launch_localize.launch.py'
        
        try:
            # สั่ง Launch RTAB-Map
            rtab_cmd = [
                'ros2', 'launch', 'my_manager', launch_file,
                f'floor:={floor_str}', f'db_name:={direction}'
            ]
            self.get_logger().info(f"🚀 Launching {mode_name.upper()}...")
            self.current_process = subprocess.Popen(rtab_cmd, preexec_fn=os.setsid)
            
            # รอให้ Process เริ่มต้นได้จริง (Check poll)
            time.sleep(3.0) 
            if self.current_process.poll() is None:
                self.get_logger().info("✅ RTAB-Map process started successfully.")
                return True
            else:
                return False

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