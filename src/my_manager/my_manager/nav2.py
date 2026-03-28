#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import os
import signal
import time
from nav2_msgs.srv import GetCostmap
from my_command.srv import Nav2 
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

class Nav2Server(Node):
    def __init__(self):
        super().__init__('nav2_manager_server')
        
        self.nav2_process = None
        self.is_active = False # ป้องกันการสั่งเปิดซ้ำซ้อน
        
        # Client สำหรับเช็คสถานะความพร้อม
        self.nav2_check_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')

        # สร้าง Service Server
        self.srv = self.create_service(
            Nav2, 
            'Nav2_service', 
            self.nav2_control_callback
        )
        
        self.get_logger().info("🚀 Nav2 Manager Server (Cleanup Enhanced) Ready.")

    def stop_nav2(self):
        """ ล้างระบบ Nav2 ทั้งหมดโดยใช้ PGID และ pkill เพื่อไม่ให้มีโหนดค้าง """
        self.get_logger().info("🧹 Cleaning up Navigation processes...")
        
        # 1. Kill Process Group ที่เก็บไว้ในตัวแปร
        if self.nav2_process:
            try:
                # ตรวจสอบว่า process ยังทำงานอยู่หรือไม่
                if self.nav2_process.poll() is None:
                    pgid = os.getpgid(self.nav2_process.pid)
                    self.get_logger().info(f"🛑 Sending SIGKILL to Nav2 PGID: {pgid}")
                    os.killpg(pgid, signal.SIGKILL)
                    self.nav2_process.wait(timeout=2.0)
            except Exception as e:
                self.get_logger().warn(f"Process cleanup notice: {e}")
            
        # 2. กวาดล้างโหนดที่มักจะค้าง (Force Kill ตามรายชื่อ)
        # กวาดล้างทั้งระบบ Nav2 และโหนดที่เกี่ยวข้องกับพารามิเตอร์
        targets = [
            'bt_navigator', 
            'controller_server', 
            'planner_server', 
            'recoveries_server',
            'waypoint_follower',
            'nav2_container',
            'amcl', # กรณีเผื่อไว้
            'lifecycle_manager'
        ]
        
        for target in targets:
            # ใช้ -f เพื่อให้ pkill หาจากชื่อเต็มของ process command
            subprocess.run(['pkill', '-9', '-f', target], stderr=subprocess.DEVNULL)
            
        self.nav2_process = None
        self.is_active = False
        self.get_logger().info("✅ All Navigation processes stopped.")
        time.sleep(1.0) # รอให้ OS เคลียร์ Port/Resource

    def wait_for_nav2_ready(self, timeout=50.0):
        """ ตรวจสอบจนกว่า Costmap Service จะปรากฏ """
        self.get_logger().info("⏳ Waiting for Nav2 Costmap to be ready...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            # ถ้า process ตายก่อนกำหนด ให้รีบแจ้ง failed
            if self.nav2_process and self.nav2_process.poll() is not None:
                self.get_logger().error("💀 Nav2 process terminated unexpectedly.")
                return False
            
            # เช็คว่า Service Global Costmap มาหรือยัง
            if self.nav2_check_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("✨ Nav2 Stack is ACTIVE and ready!")
                return True
            
            self.get_logger().info("... still waiting for Nav2 ...")
            
        return False

    def nav2_control_callback(self, request, response):
        # กรณีสั่ง Stop หรือสั่ง Active=False
        if not request.active:
            self.stop_nav2()
            response.success = True
            return response

        # กรณีสั่ง Active=True
        if self.is_active:
            self.get_logger().info("ℹ️ Nav2 is already running.")
            response.success = True
            return response

        self.get_logger().info("📥 Request received: Starting Navigation Stack")
        
        # 1. ปิดของเก่าก่อน (ถ้ามี)
        self.stop_nav2()

        # 2. หา Path ของ params_file
        try:
            pkg_path = get_package_share_directory('my_manager')
            nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
        except:
            nav2_params = '/home/noone/Robotics_Project/src/my_manager/config/nav2_params.yaml'

        # 3. เตรียมคำสั่ง Launch
        nav2_command = [
            'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
            'use_sim_time:=false',
            f'params_file:={nav2_params}',
            'use_amcl:=false', 
            'map:=/rtabmap/map'
        ]

        try:
            self.get_logger().info(f"🚀 Launching Nav2 with params: {nav2_params}")
            # ใช้ os.setsid เพื่อสร้าง Process Group ใหม่ ทำให้ Kill ได้ทั้งกลุ่ม
            self.nav2_process = subprocess.Popen(nav2_command, preexec_fn=os.setsid)
            
            # 4. รอจนกว่าระบบจะเปิดติดจริง
            if self.wait_for_nav2_ready(timeout=50.0):
                self.is_active = True
                response.success = True
            else:
                self.stop_nav2() # ถ้าเปิดไม่ติดในเวลาที่กำหนด ให้ Kill ทิ้งทันที
                response.success = False
                
        except Exception as e:
            self.get_logger().error(f"❌ Launch System Error: {e}")
            self.stop_nav2()
            response.success = False
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Server()
    
    # ใช้ MultiThreadedExecutor เพื่อให้ Service callback ทำงานได้ราบรื่น
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Node interrupted, cleaning up...")
    finally:
        node.stop_nav2() # ปิด process ทั้งหมดก่อนจบโปรแกรม
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()