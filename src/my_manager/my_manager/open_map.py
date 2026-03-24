#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess
import threading
import time
import os
import signal
from my_command.srv import OpenMap # Interface: string mode, string way, float floor -> string status
from ament_index_python.packages import get_package_share_directory 

class OpenMapServer(Node):
    def __init__(self):
        super().__init__('open_map_server')
        
        # เก็บ Process แยกกันเพื่อให้จัดการได้แม่นยำ
        self.current_process = None # สำหรับ RTAB-Map Launch
        self.nav2_process = None    # สำหรับ Nav2 Launch
        
        self.current_launch_id = "" 
        self.should_show_pose = False

        # สร้าง Service Server
        self.srv = self.create_service(
            OpenMap, 
            'open_map_service', 
            self.open_map_callback
        )
        
        self.get_logger().info("🤖 OpenMap Service Server Ready (with Nav2 Support).")

    def open_map_callback(self, request, response):
        """
        Callback เมื่อมีการเรียก Service open_map_service
        """
        mode_val = request.mode  # 'localize' หรือ 'map'
        way_val = request.way    # 'go' หรือ 'back'
        floor_num = request.floor
        
        floor_str = f"floor{int(floor_num)}"
        launch_id = f"{mode_val}_{way_val}_{floor_str}"

        # ถ้าเป็นแผนที่เดิมที่รันอยู่แล้ว ไม่ต้องรันใหม่
        if self.current_launch_id == launch_id:
            self.get_logger().info(f"ℹ️ {launch_id} is already running.")
            response.status = "success"
            return response

        self.get_logger().info(f"📥 Service Request: Mode={mode_val}, Floor={floor_str}, Way={way_val}")
        
        # เรียกฟังก์ชันจัดการรัน Launch
        success = self.run_launch(mode_val, floor_str, way_val)
        
        if success:
            self.current_launch_id = launch_id
            response.status = "success"
        else:
            response.status = "failed"
            
        return response

    def run_launch(self, mode_name, floor, direction):
        """
        ฟังก์ชันจัดการการปิดตัวเก่าและรัน RTAB-Map + Nav2 ใหม่
        """
        # 1. เคลียร์ Process เก่าทั้งหมดก่อนเริ่มใหม่ (ป้องกันซ้อนทับ)
        for proc in [self.current_process, self.nav2_process]:
            if proc:
                try:
                    self.get_logger().info(f"🛑 Terminating process group: {proc.pid}")
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    time.sleep(1.0)
                    os.killpg(pgid, signal.SIGKILL)
                except:
                    pass
        
        self.current_process = None
        self.nav2_process = None

        # 2. Cleanup GUI และสิ่งตกค้าง
        try:
            subprocess.run(['pkill', '-9', 'rtabmap_viz'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', 'rviz2'], stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        except:
            pass

        # 3. เริ่มรัน RTAB-Map
        launch_file = 'launch_mapping.launch.py' if mode_name == 'map' else 'launch_localize.launch.py'
        rtab_command = [
            'ros2', 'launch', 'my_manager', launch_file,
            f'floor:={floor}',
            f'db_name:={direction}'
        ]
        
        try:
            self.get_logger().info(f"🚀 Launching RTAB: {mode_name.upper()} | Floor: {floor}")
            self.current_process = subprocess.Popen(rtab_command, preexec_fn=os.setsid)
            
            # รอ 5 วินาทีให้ RTAB-Map และกล้องตั้งตัว
            time.sleep(5.0) 
            
            if self.current_process.poll() is not None:
                self.get_logger().error("❌ RTAB-Map failed to start.")
                return False

            # 4. เริ่มรัน Nav2 หลังจาก RTAB-Map พร้อมแล้ว
            try:
                # พยายามดึง path จาก package share
                pkg_path = get_package_share_directory('my_manager')
                nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
            except:
                # กรณีฉุกเฉินหาไฟล์ไม่เจอ ใช้ path ตรงๆ
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
            
            # รอ Nav2 อีกสักพักก่อนยืนยันความสำเร็จ
            time.sleep(3.0)
            
            self.get_logger().info(f"✅ Full System ({mode_name.upper()} + Nav2) Started successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"❌ Launch System Error: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = OpenMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # สั่งหยุดทุกลูกข่ายเมื่อปิด Node หลัก
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