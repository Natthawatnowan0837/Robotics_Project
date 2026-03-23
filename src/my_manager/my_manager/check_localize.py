#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rtabmap_msgs.msg import Info
from std_msgs.msg import String
import threading
import time
from my_command.srv import CheckLocalize # Interface: bool active -> bool success

class Check_Localize(Node):
    def __init__(self):
        super().__init__('auto_search_localize')
        
        # ใช้ Callback Group เพื่อให้ Service และ Subscriptions ทำงานขนานกันได้
        self.group = ReentrantCallbackGroup()

        # --- [ States ] ---
        self.is_localized = False
        self.system_activated = False 
        self.rtabmap_ready = False 
        self.startup_delay = 3.0 
        
        self.internal_step = 0 
        self.waiting_for_status = False 
        self.wait_counter = 0

        # ตัวแปรสำหรับเก็บ Service Response
        self.current_service_request = None

        # --- [ Publishers ] ---
        self.seq_pub = self.create_publisher(String, 'sequence_cmd', 10)
        
        # --- [ Subscriptions ] ---
        self.create_subscription(Info, '/rtabmap/info', self.info_callback, 10, callback_group=self.group)
        self.create_subscription(String, 'sequence_status', self.status_callback, 10, callback_group=self.group)

        # --- [ Service Server ] ---
        self.srv = self.create_service(
            CheckLocalize, 
            'check_localization_service', 
            self.handle_check_localize,
            callback_group=self.group
        )

        # Main Loop (1Hz)
        self.create_timer(1.0, self.search_logic_loop, callback_group=self.group)
        
        self.get_logger().info('🎯 Auto Search Service Server Ready.')

    def handle_check_localize(self, request, response):
        """ Callback เมื่อมีการเรียก Service """
        if request.active:
            self.get_logger().info("📥 Service Called: Activating Search Pattern...")
            
            # Reset ค่าก่อนเริ่มค้นหาใหม่
            self.reset_internal_logic()
            
            # รอ 3 วินาที (แบบ Blocking ภายใน Service Thread นี้เพื่อรอผลลัพธ์)
            time.sleep(self.startup_delay)
            self.system_activated = True
            
            # วนลูปตรวจสอบจนกว่าจะ Localize สำเร็จ หรือ Timeout (ถ้ามี)
            # ในที่นี้จะรอจนกว่า info_callback จะตั้งค่า self.is_localized = True
            while not self.is_localized:
                time.sleep(0.5)
                # คุณอาจเพิ่มเงื่อนไขหลุดลูปตรงนี้หากรอนานเกินไป
            
            response.success = True
            self.get_logger().info("📤 Sending Service Response: Localization Success!")
            return response
        else:
            self.system_activated = False
            response.success = False
            return response

    def reset_internal_logic(self):
        self.is_localized = False
        self.internal_step = 0
        self.wait_counter = 0
        self.waiting_for_status = False
        self.system_activated = False

    def info_callback(self, msg):
        self.rtabmap_ready = True
        # เช็คว่าเจอ Loop Closure หรือ Proximity หรือยัง
        if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
            if not self.is_localized:
                self.get_logger().info("🎯 [MATCH FOUND] RTAB-Map Localized!")
                self.is_localized = True
                self.system_activated = False 

    def status_callback(self, msg):
        # รับสถานะการเคลื่อนที่จาก Robot
        if msg.data == "1" or msg.data.upper() == "DONE":
            if self.waiting_for_status:
                self.get_logger().info("✅ Movement Done.")
                self.waiting_for_status = False

    def search_logic_loop(self):
        """ Loop สำหรับสั่งหุ่นยนต์เคลื่อนที่ไปรอบๆ เพื่อหาตำแหน่ง """
        if not self.system_activated or self.is_localized:
            return

        if not self.rtabmap_ready:
            self.get_logger().warn("⏳ Waiting for RTAB-Map info...", throttle_duration_sec=5.0)
            return

        if self.waiting_for_status:
            return

        # Simple Search State Machine
        if self.internal_step == 0:
            self.get_logger().info("📤 Step 0: Executing FWD")
            self.send_cmd("fwd")
            self.internal_step = 1 
        elif self.internal_step == 1:
            self.get_logger().info("📤 Step 1: Executing LEFT180")
            self.send_cmd("left180")
            self.internal_step = 2 
        elif self.internal_step == 2:
            self.get_logger().info("📤 Step 2: Executing FWD (Return)")
            self.send_cmd("fwd")
            self.internal_step = 0

    def send_cmd(self, command_str):
        msg = String(data=command_str)
        self.seq_pub.publish(msg)
        self.waiting_for_status = True 

def main():
    rclpy.init()
    node = Check_Localize()
    
    # สำคัญมาก: ต้องใช้ MultiThreadedExecutor เพราะมีการรอ (while/sleep) ใน Service
    executor = MultiThreadedExecutor()
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