#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from std_msgs.msg import String
import threading # เพิ่มเพื่อใช้ Timer

class AutoSearchLocalize(Node):
    def __init__(self):
        super().__init__('auto_search_localize')
        
        # --- [ Configurations / States ] ---
        self.is_localized = False
        self.search_active = False 
        self.rtabmap_ready = False 
        self.system_activated = False 
        self.startup_delay = 3.0 
        
        # เพิ่มตัวแปรสำหรับควบคุมการรอ 3 วินาที
        self.is_waiting_delay = False
        self.startup_timer = None

        self.internal_step = 0 
        self.waiting_for_status = False 
        self.is_checking_spot = False   
        self.wait_counter = 0

        # --- [ Publishers ] ---
        self.seq_pub = self.create_publisher(String, 'sequence_cmd', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)
        
        # --- [ Subscriptions ] ---
        self.create_subscription(Info, '/rtabmap/info', self.info_callback, 10)
        self.create_subscription(String, 'sequence_status', self.status_callback, 10)
        self.create_subscription(String, 'action', self.activate_callback, 10)

        # Main Loop (1Hz)
        self.create_timer(1.0, self.search_logic_loop)
        
        self.get_logger().info('🟢 Auto Search Node Ready (Subscribing to "action" string)')

    def activate_callback(self, msg):
        try:
            raw_data = msg.data.strip().lower()
            
            if raw_data == "action":
                # 1. Reset ค่าสถานะภายในเสมอเมื่อเริ่มรอบใหม่
                self.reset_internal_logic()

                # 2. จัดการเรื่อง Timer (ถ้ามีของเก่าที่กำลังนับถอยหลังอยู่ให้ยกเลิกก่อน)
                if self.startup_timer is not None:
                    self.startup_timer.cancel()
                
                self.get_logger().info(f"⏳ Received 'action'. Waiting {self.startup_delay}s delay...")
                self.is_waiting_delay = True
                
                # 3. เริ่มนับถอยหลัง 3 วินาที (แบบไม่บล็อก Node)
                self.startup_timer = threading.Timer(self.startup_delay, self.start_system)
                self.startup_timer.start()
            
            elif raw_data in ["stop", "nav2", "position"]:
                self.system_activated = False
                self.search_active = False
                self.is_waiting_delay = False
                if self.startup_timer is not None:
                    self.startup_timer.cancel()
                self.get_logger().info(f"💤 Received '{raw_data}': Search pattern deactivated.")

        except Exception as e:
            self.get_logger().error(f"❌ Error parsing action message: {e}")

    def start_system(self):
        """ ฟังก์ชันนี้จะทำงานหลังจากผ่านไป 3 วินาที """
        self.get_logger().info("🚀 Startup delay finished! Starting search pattern...")
        self.system_activated = True
        self.search_active = True
        self.is_waiting_delay = False

    def reset_internal_logic(self):
        """ ล้างค่าตัวแปร Logic ภายในเพื่อเริ่มค้นหาใหม่ """
        self.is_localized = False
        self.internal_step = 0
        self.wait_counter = 0
        self.is_checking_spot = False
        self.waiting_for_status = False
        self.system_activated = False # รอจนกว่า Timer จะสั่ง True

    def info_callback(self, msg):
        self.rtabmap_ready = True
        if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
            if not self.is_localized:
                self.get_logger().info("🎯 [MATCH FOUND] Localized successfully!")
                self.is_localized = True
                self.search_active = False
                self.system_activated = False 
                
                status_msg = String(data="action,done")
                self.status_pub.publish(status_msg)
                self.get_logger().info(f"📤 Published status: {status_msg.data}")

    def status_callback(self, msg):
        if msg.data == "1" or msg.data.upper() == "DONE":
            if self.waiting_for_status:
                self.get_logger().info("✅ Movement Done. Starting Spot Check...")
                self.waiting_for_status = False
                self.wait_counter = 0

    def search_logic_loop(self):
        # --- เงื่อนไขการข้าม Loop ---
        # ข้ามถ้า: ยังอยู่ในช่วงรอ 3 วิ / ระบบยังไม่เริ่ม / หรือ Localize ได้แล้ว
        if self.is_waiting_delay or not self.system_activated or self.is_localized:
            return

        if not self.rtabmap_ready:
            self.get_logger().warn("⏳ Waiting for RTAB-Map info...", throttle_duration_sec=5.0)
            return

        if self.waiting_for_status:
            return

        if self.is_checking_spot:
            self.wait_counter += 1
            if self.wait_counter <= 5:
                return
            else:
                self.get_logger().info("❌ No match found. Moving next...")
                self.is_checking_spot = False 

        # --- State Machine เคลื่อนที่ตามปกติ ---
        if self.internal_step == 0:
            self.get_logger().info("📤 Step 0: Executing FWD")
            self.send_cmd("fwd")
            self.is_checking_spot = True 
            self.internal_step = 1 
        elif self.internal_step == 1:
            self.get_logger().info("📤 Step 1: Executing LEFT180")
            self.send_cmd("left180")
            self.internal_step = 2 
        elif self.internal_step == 2:
            self.get_logger().info("📤 Step 2: Executing FWD (Return)")
            self.send_cmd("fwd")
            self.is_checking_spot = True 
            self.internal_step = 0

    def send_cmd(self, command_str):
        msg = String(data=command_str)
        self.seq_pub.publish(msg)
        self.waiting_for_status = True 

def main():
    rclpy.init()
    node = AutoSearchLocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()