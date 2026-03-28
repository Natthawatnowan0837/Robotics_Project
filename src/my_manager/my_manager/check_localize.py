#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rtabmap_msgs.msg import Info
import time

# นำเข้า Service Interfaces
from my_command.srv import CheckLocalize
from my_command.srv import SequenceCmd

class CheckLocalizeNode(Node):
    def __init__(self):
        super().__init__('auto_search_localize')
        
        self.group = ReentrantCallbackGroup()

        # --- [ States ] ---
        self.is_localized = False
        self.system_activated = False 
        self.rtabmap_ready = False 
        self.internal_step = 0 # 0: Check, 1: Rotate
        self.is_waiting_for_service = False 
        self.check_start_time = 0.0

        # --- [ Service Clients ] ---
        # เปลี่ยนชื่อ Service ให้ตรงกับโหนดหมุนที่เราทำ (rotate_service)
        self.sequence_client = self.create_client(
            SequenceCmd, 'rotate_service', callback_group=self.group)

        # --- [ Subscriptions ] ---
        self.create_subscription(
            Info, '/rtabmap/info', self.info_callback, 10, callback_group=self.group)

        # --- [ Service Server ] ---
        self.srv = self.create_service(
            CheckLocalize, 'check_localization_service', 
            self.handle_check_localize, callback_group=self.group)

        # Main Loop (1Hz)
        self.create_timer(1.0, self.search_logic_loop, callback_group=self.group)
        
        self.get_logger().info('🎯 Auto Search Node Ready (Check -> Rotate 180).')

    def info_callback(self, msg):
        self.rtabmap_ready = True
        # เช็คว่าเจอตำแหน่งเดิมหรือยัง (Loop Closure / Proximity)
        if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
            if not self.is_localized:
                self.get_logger().info("🎯 [MATCH FOUND] RTAB-Map Localized!")
                self.is_localized = True
                self.system_activated = False # หยุดการวนลูปค้นหา

    def handle_check_localize(self, request, response):
        if request.active:
            self.get_logger().info("📥 Service Called: Starting Search Pattern...")
            self.is_localized = False
            self.internal_step = 0 # เริ่มที่การเช็คก่อน
            self.system_activated = True
            
            # รอจนกว่าจะ Localize เจอ (Blocking)
            while rclpy.ok() and not self.is_localized and self.system_activated:
                time.sleep(0.5)
            
            response.success = self.is_localized
            return response
        else:
            self.system_activated = False
            response.success = False
            return response

    async def search_logic_loop(self):
        # เงื่อนไขการหยุดทำงาน
        if not self.system_activated or self.is_localized or self.is_waiting_for_service:
            return

        if not self.rtabmap_ready:
            self.get_logger().warn("⏳ Waiting for RTAB-Map info...", throttle_duration_sec=5.0)
            return

        # --- STEP 0: ยืนรอเช็คตำแหน่ง (3-5 วินาที) ---
        if self.internal_step == 0:
            if self.check_start_time == 0.0:
                self.get_logger().info("🔍 Step 0: Checking Localization (Standing Still)...")
                self.check_start_time = time.time()
            
            elapsed = time.time() - self.check_start_time
            if elapsed > 5.0: # ถ้ายืนรอ 5 วินาทีแล้วยังไม่เจอ
                self.get_logger().info("❌ Still not localized. Moving to Step 1.")
                self.internal_step = 1
                self.check_start_time = 0.0 # รีเซ็ตเวลา
            return

        # --- STEP 1: ส่งคำสั่งหมุน 180 องศา ---
        if self.internal_step == 1:
            if not self.sequence_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("⚠️ Rotate Service not available!")
                return

            self.is_waiting_for_service = True
            self.get_logger().info("📤 Step 1: Sending 'left180' to Rotate Service...")

            req = SequenceCmd.Request()
            req.state = "right180" # คำสั่งให้โหนดหมุนทำงาน

            try:
                future = self.sequence_client.call_async(req)
                result = await future # รอจนกว่าโหนดหมุนจะตอบกลับ (SUCCESS)
                
                if result is not None:
                    self.get_logger().info(f"✅ Rotate Finished. Returning to Step 0.")
                    self.internal_step = 0 # กลับไปเช็คใหม่
            except Exception as e:
                self.get_logger().error(f"❌ Rotation failed: {e}")
            finally:
                self.is_waiting_for_service = False

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = CheckLocalizeNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()