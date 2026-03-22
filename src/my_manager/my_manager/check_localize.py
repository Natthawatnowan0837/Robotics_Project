#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from std_msgs.msg import String, Int32 # เพิ่ม Int32 เข้ามา

class AutoSearchLocalize(Node):
    def __init__(self):
        super().__init__('auto_search_localize')
        
        # --- [ Configurations ] ---
        self.is_localized = False
        self.search_active = False 
        self.rtabmap_ready = False 
        self.system_activated = False 
        
        # internal_step: 0=FWD, 1=LEFT180, 2=FWD(After Turn)
        self.internal_step = 0 
        
        self.waiting_for_status = False 
        self.is_checking_spot = False   
        self.wait_counter = 0

        # --- [ Pub/Sub ] ---
        self.seq_pub = self.create_publisher(String, 'sequence_cmd', 10)
        
        self.create_subscription(Info, '/rtabmap/info', self.info_callback, 10)
        self.create_subscription(String, 'sequence_status', self.status_callback, 10)
        
        # --- [ แก้ไข Topic ให้รับเป็น Int32 ] ---
        self.create_subscription(Int32, 'active_check_localize', self.activate_callback, 10)

        self.create_timer(1.0, self.search_logic_loop)
        self.get_logger().info('🟢 Auto Search Node Ready (Waiting for 1 on /active_check_localize)')

    def activate_callback(self, msg):
        """ รับค่า 1 เพื่อเริ่ม และ 0 เพื่อหยุด """
        cmd = msg.data
        if cmd == 1:
            if not self.system_activated:
                self.system_activated = True
                self.search_active = True
                self.get_logger().info("🚀 [SYSTEM ACTIVATED] Received 1: Starting localization search...")
        elif cmd == 0:
            if self.system_activated:
                self.system_activated = False
                self.search_active = False
                self.get_logger().warn("🛑 [SYSTEM DEACTIVATED] Received 0: Search stopped.")

    def info_callback(self, msg):
        self.rtabmap_ready = True
        # เช็คทั้ง Loop Closure และ Proximity Detection (การเจอจุดใกล้เคียง)
        if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
            if not self.is_localized:
                self.get_logger().info("🎯 [MATCH FOUND] Localized! Stopping all search.")
            self.is_localized = True
            self.search_active = False

    def status_callback(self, msg):
        # รอรับสถานะว่าหุ่นยนต์ทำงาน sequence เสร็จหรือยัง
        if msg.data == "1" or msg.data.upper() == "DONE":
            if self.waiting_for_status:
                self.get_logger().info("✅ Robot reached position. Starting Spot Check...")
                self.waiting_for_status = False
                self.is_checking_spot = True 
                self.wait_counter = 0

    def search_logic_loop(self):
        # 1. เช็คว่าระบบถูกสั่งรัน (1) หรือยัง
        if not self.system_activated:
            return

        # 2. เช็คว่าเจอตำแหน่งหรือยัง
        if not self.search_active or self.is_localized:
            return

        # 3. เช็คสถานะ RTAB-Map
        if not self.rtabmap_ready:
            self.get_logger().warn("⏳ Waiting for RTAB-Map info...", throttle_duration_sec=5.0)
            return

        # 4. ถ้าหุ่นกำลังวิ่ง (รอ Status จาก Arduino/Sequence) ให้รอ
        if self.waiting_for_status:
            return

        # 5. Spot Check (หยุดนิ่งเพื่อให้ RTAB-Map เทียบภาพได้ชัดๆ)
        if self.is_checking_spot:
            self.wait_counter += 1
            if self.wait_counter <= 10:
                self.get_logger().info(f"🧐 Spot Checking... ({self.wait_counter}/10s)", throttle_duration_sec=1.0)
                return
            else:
                self.get_logger().info("❌ No match found. Moving to next step.")
                self.is_checking_spot = False 

        # 6. --- [ State Machine Logic ] ---
        if self.internal_step == 0:
            self.get_logger().info("📤 Executing: FWD")
            self.send_cmd("fwd")
            self.internal_step = 1 

        elif self.internal_step == 1:
            self.get_logger().info("📤 Executing: LEFT180")
            self.send_cmd("left180")
            self.internal_step = 2 

        elif self.internal_step == 2:
            self.get_logger().info("📤 Executing: FWD (After Turn)")
            self.send_cmd("fwd")
            self.internal_step = 0 

    def send_cmd(self, command_str):
        msg = String()
        msg.data = command_str
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