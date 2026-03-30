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
        # แก้ไขลำดับ Step: -1: เดินหน้า, 0: Check, 1: Rotate
        self.internal_step = -1 
        self.is_waiting_for_service = False 
        self.check_start_time = 0.0

        # --- [ Service Clients ] ---
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
        
        self.get_logger().info('🎯 Auto Search Node Ready (Forward -> Check -> Rotate).')

    def info_callback(self, msg):
        self.rtabmap_ready = True
        if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
            if not self.is_localized and self.system_activated:
                self.get_logger().info("🎯 [MATCH FOUND] RTAB-Map Localized! Sending STOP command...")
                self.is_localized = True
                self.system_activated = False 
                self.send_stop_command()

    def send_stop_command(self):
        if not self.sequence_client.wait_for_service(timeout_sec=1.0):
            return
        req = SequenceCmd.Request()
        req.state = "stop"
        self.get_logger().info("🛑 Sending 'stop' to Rotate Service...")
        self.sequence_client.call_async(req)

    def handle_check_localize(self, request, response):
        if request.active:
            self.get_logger().info("📥 Service Called: Starting Search Pattern...")
            self.is_localized = False
            self.internal_step = -1 # เริ่มต้นที่ Step เดินหน้าก่อนเสมอ
            self.system_activated = True
            
            while rclpy.ok() and not self.is_localized and self.system_activated:
                time.sleep(0.2)
            
            response.success = self.is_localized
            return response
        else:
            self.system_activated = False
            self.send_stop_command()
            response.success = False
            return response

    async def search_logic_loop(self):
        if not self.system_activated or self.is_localized or self.is_waiting_for_service:
            return

        if not self.rtabmap_ready:
            self.get_logger().warn("⏳ Waiting for RTAB-Map info...", throttle_duration_sec=5.0)
            return

        # --- [ NEW ] STEP -1: ส่งคำสั่งเดินหน้า (Forward) ---
        if self.internal_step == -1:
            if not self.sequence_client.wait_for_service(timeout_sec=1.0):
                return

            self.is_waiting_for_service = True
            self.get_logger().info("📤 Step -1: Sending 'fwd' before localization...")

            req = SequenceCmd.Request()
            req.state = "fwd" 

            try:
                future = self.sequence_client.call_async(req)
                await future 
                if self.system_activated:
                    self.get_logger().info("✅ Forward finished. Moving to Step 0 (Check).")
                    self.internal_step = 0 # เดินหน้าเสร็จแล้วไปยืนเช็คต่อ
            except Exception as e:
                self.get_logger().error(f"❌ Forward call failed: {e}")
            finally:
                self.is_waiting_for_service = False
            return

        # --- STEP 0: ยืนรอเช็คตำแหน่ง (Standing Still) ---
        if self.internal_step == 0:
            if self.check_start_time == 0.0:
                self.get_logger().info("🔍 Step 0: Checking Localization (Standing Still)...")
                self.check_start_time = time.time()
            
            elapsed = time.time() - self.check_start_time
            if elapsed > 5.0: 
                self.get_logger().info("❌ Still not localized. Moving to Step 1 (Rotate).")
                self.internal_step = 1
                self.check_start_time = 0.0 
            return

        # --- STEP 1: ส่งคำสั่งหมุน (Rotate) ---
        if self.internal_step == 1:
            if not self.sequence_client.wait_for_service(timeout_sec=1.0):
                return

            self.is_waiting_for_service = True
            self.get_logger().info("📤 Step 1: Sending 'right180' to Rotate Service...")

            req = SequenceCmd.Request()
            req.state = "right180" 

            try:
                future = self.sequence_client.call_async(req)
                await future 
                if self.system_activated:
                    self.get_logger().info("✅ Rotate sequence finished. Returning to Step 0.")
                    self.internal_step = 0 
            except Exception as e:
                self.get_logger().error(f"❌ Rotation call failed: {e}")
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