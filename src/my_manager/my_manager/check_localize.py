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
        self.internal_step = 0 
        self.is_waiting_for_service = False # ตัวป้องกัน Timer เรียกซ้ำ

        # --- [ Service Clients ] ---
        self.sequence_client = self.create_client(
            SequenceCmd, 'sequence_cmd_service', callback_group=self.group)

        # --- [ Subscriptions ] ---
        self.create_subscription(
            Info, '/rtabmap/info', self.info_callback, 10, callback_group=self.group)

        # --- [ Service Server ] ---
        self.srv = self.create_service(
            CheckLocalize, 'check_localization_service', 
            self.handle_check_localize, callback_group=self.group)

        # Main Loop (1Hz)
        self.create_timer(1.0, self.search_logic_loop, callback_group=self.group)
        
        self.get_logger().info('🎯 Auto Search Node Ready.')

    def info_callback(self, msg):
        self.rtabmap_ready = True
        if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
            if not self.is_localized:
                self.get_logger().info("🎯 [MATCH FOUND] RTAB-Map Localized!")
                self.is_localized = True
                self.system_activated = False 

    def handle_check_localize(self, request, response):
        if request.active:
            self.get_logger().info("📥 Service Called: Starting Search Pattern...")
            self.is_localized = False
            self.internal_step = 0
            time.sleep(2.0)
            self.system_activated = True
            
            while rclpy.ok() and not self.is_localized:
                time.sleep(0.5)
            
            response.success = True
            return response
        else:
            self.system_activated = False
            response.success = False
            return response

    async def search_logic_loop(self):
        # ป้องกันการรันซ้อนถ้าหุ่นยังขยับไม่เสร็จ หรือ Localize เจอแล้ว
        if not self.system_activated or self.is_localized or self.is_waiting_for_service:
            return

        if not self.rtabmap_ready:
            self.get_logger().warn("⏳ Waiting for RTAB-Map info...", throttle_duration_sec=5.0)
            return

        if not self.sequence_client.wait_for_service(timeout_sec=1.0):
            return

        # กำหนด Step การเดิน
        cmd = "fwd" if self.internal_step in [0, 2] else "left180"
        next_step = (self.internal_step + 1) % 3

        # ล็อคสถานะและส่งคำสั่ง
        self.is_waiting_for_service = True
        self.get_logger().info(f"📤 Requesting: {cmd} (Step {self.internal_step})")

        req = SequenceCmd.Request()
        req.active = "true"
        req.state = cmd

        try:
            future = self.sequence_client.call_async(req)
            result = await future # หยุดรอตรงนี้จนกว่าโหนดลูกจะตอบ DONE
            
            if result is not None:
                self.get_logger().info(f"✅ Finished: {result.status}")
                self.internal_step = next_step
        except Exception as e:
            self.get_logger().error(f"❌ Service failed: {e}")
        finally:
            self.is_waiting_for_service = False

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(CheckLocalizeNode())
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()