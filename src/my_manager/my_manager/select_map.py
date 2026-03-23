#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess
import threading
import time
import os      # <--- แทรก: เพื่อจัดการ Process ID
import signal  # <--- แทรก: เพื่อส่งสัญญาณ Kill ยกกลุ่ม

class LaunchSwitcher(Node):
    def __init__(self):
        super().__init__('launch_switcher')
        
        # 1. Subscribe รับข้อมูล [mode, way, floor]
        self.sub_opening = self.create_subscription(
            Float32MultiArray,
            'opening',
            self.opening_callback,
            10)
    
        # 2. Subscribe รับคำสั่งให้เช็คตำแหน่ง
        self.sub_check_pos = self.create_subscription(
            String,
            'position',
            self.position_trigger_callback,
            10)

        # 3. Subscribe รอรับค่าพิกัดจริงจาก RTAB-Map
        self.sub_rtab_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.rtab_pose_callback,
            10)
        
        # Publisher สำหรับส่งสถานะกลับ
        self.status_pub = self.create_publisher(String, 'status', 10)
        
        self.current_process = None
        self.current_launch_id = "" 
        self.should_show_pose = False

        self.get_logger().info("🤖 Launch Switcher Ready.")

    def opening_callback(self, msg):
        if len(msg.data) >= 3:
            mode_val = int(msg.data[0])
            way_val = int(msg.data[1])
            floor_num = int(msg.data[2])
            
            direction = 'go' if way_val == 0 else 'back'
            floor_str = f"floor{floor_num}"
            mode_name = "map" if mode_val == 0 else "localize"
            
            launch_id = f"{mode_name}_{direction}_{floor_str}"
            if self.current_launch_id == launch_id:
                return 

            self.current_launch_id = launch_id
            thread = threading.Thread(target=self.run_launch, args=(mode_name, floor_str, direction))
            thread.start()

    def position_trigger_callback(self, msg):
        if msg.data.lower() == "check":
            self.get_logger().info("🔍 Position check requested. Waiting for RTAB-Map pose...")
            self.should_show_pose = True

    def rtab_pose_callback(self, msg):
        if self.should_show_pose:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(f"📍 CURRENT POSITION -> X: {x:.2f}, Y: {y:.2f}")
            self.should_show_pose = False 

    # --- ส่วนที่แก้ไข: การจัดการกลุ่ม Process เพื่อปิด Rviz และ Node ลูกทั้งหมด ---
    def run_launch(self, mode_name, floor, direction):
            """
            ฟังก์ชันสำหรับปิดตัวเก่าและรัน Launch ใหม่
            โดยเน้นการปิด rviz2 และ rtabmap_viz ให้สนิท
            """
            # 1. ตรวจสอบและปิด Process เดิมที่รันผ่านตัวแปร current_process (ถ้ามี)
            if self.current_process:
                self.get_logger().warn(f"🛑 Terminating previous group: {self.current_launch_id}...")
                try:
                    # ส่งสัญญาณ SIGTERM (ปิดแบบสุภาพ) ไปยังกลุ่ม Process ทั้งหมด
                    pgid = os.getpgid(self.current_process.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    
                    # ให้เวลาโปรแกรมเคลียร์ฐานข้อมูล .db เล็กน้อย (1.5 วินาที)
                    time.sleep(1.5)
                    
                    # ส่งสัญญาณ SIGKILL (บังคับปิดทันที) เพื่อจัดการตัวที่ยังดื้อไม่ยอมปิด
                    os.killpg(pgid, signal.SIGKILL)
                    self.current_process.wait(timeout=2)
                except Exception as e:
                    self.get_logger().error(f"⚠️ Error killing process group: {e}")

            # 2. [ไม้ตายสุดท้าย] สั่งกวาดล้าง rtabmap_viz และ rviz2 ที่อาจจะหลุดกลุ่ม
            # คำสั่ง pkill -9 จะบังคับปิด Process ตามชื่อทันที
            try:
                self.get_logger().info("🧹 Cleaning up lingering GUI processes...")
                subprocess.run(['pkill', '-9', 'rtabmap_viz'], stderr=subprocess.DEVNULL)
                subprocess.run(['pkill', '-9', 'rviz2'], stderr=subprocess.DEVNULL)
                # รอให้ระบบคืนทรัพยากรครู่หนึ่ง
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f"Cleanup error: {e}")

            # 3. เตรียมคำสั่ง Launch ใหม่
            # เลือกไฟล์ตามโหมด 0=map, 1=localize
            launch_file = 'launch_mapping.launch.py' if mode_name == 'map' else 'launch_localize.launch.py'
            
            command = [
                'ros2', 'launch', 'my_manager', launch_file,
                f'floor:={floor}',
                f'db_name:={direction}'
            ]
            
            try:
                self.get_logger().info(f"🚀 Launching new: {mode_name.upper()} at {floor} ({direction})")
                
                # 4. รัน Subprocess โดยใช้ os.setsid เพื่อสร้าง Process Group ID ใหม่
                # วิธีนี้จะทำให้เราสามารถสั่งปิดยกกลุ่มได้ในครั้งถัดไป
                self.current_process = subprocess.Popen(
                    command, 
                    preexec_fn=os.setsid
                )
                
                # รอให้ระบบ Launch เริ่มต้น (เช่น 4 วินาที)
                time.sleep(4.0)
                
                # เช็คว่า Process ยังอยู่ดี (ไม่ได้ Crash ทันที)
                if self.current_process.poll() is None:
                    status_msg = String()
                    status_msg.data = f"{mode_name},done"
                    self.status_pub.publish(status_msg)
                    self.get_logger().info(f"✅ Success: Published [{status_msg.data}]")
                else:
                    self.get_logger().error(f"❌ Failed to start {mode_name}. Process exited early.")
                    
            except Exception as e:
                self.get_logger().error(f"❌ Subprocess Execution Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaunchSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # เมื่อปิด Node หลัก ให้ตามไปปิดกลุ่ม Process ที่รันค้างไว้ด้วย
        if node.current_process:
            try:
                os.killpg(os.getpgid(node.current_process.pid), signal.SIGTERM)
            except:
                pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()