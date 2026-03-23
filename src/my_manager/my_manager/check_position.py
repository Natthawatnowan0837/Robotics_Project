#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
import time
from my_command.srv import CheckPosition # Interface: float32 x, float32 y, string way -> string update_way

class CheckPositionServer(Node):
    def __init__(self):
        super().__init__('check_position_node')
        
        self.group = ReentrantCallbackGroup()

        # --- Publishers ---
        self.pub_final_goal = self.create_publisher(Float32MultiArray, 'final_goal', 10)
        self.seq_pub = self.create_publisher(String, 'sequence_cmd', 10)
        
        # --- Subscriptions ---
        self.sub_rtab_pose = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/rtabmap/localization_pose', 
            self.rtab_pose_callback, 
            10,
            callback_group=self.group)
            
        self.sub_seq_status = self.create_subscription(
            String, 
            'sequence_status', 
            self.status_callback, 
            10,
            callback_group=self.group)

        # --- Service Server ---
        self.srv = self.create_service(
            CheckPosition, 
            'check_position_service', 
            self.handle_check_position,
            callback_group=self.group)

        # --- Variables ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.waiting_for_status = False
        self.done_event = threading.Event() # ใช้สำหรับรอสถานะการหมุน

        self.get_logger().info("🚀 Check Position Service Server Ready.")

    def rtab_pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def status_callback(self, msg):
        if msg.data == "1" or msg.data.upper() == "DONE":
            if self.waiting_for_status:
                self.get_logger().info("✅ Rotation Done.")
                self.waiting_for_status = False
                self.done_event.set() # แจ้งให้ Service Callback ทราบว่าหมุนเสร็จแล้ว

    def handle_check_position(self, request, response):
        """ 
        Logic เปรียบเทียบตำแหน่งตามสมการฟังก์ชัน 
        request.x, request.y, request.way ('go' หรือ 'back')
        """
        self.get_logger().info(f"🔍 Checking: Robot_X({self.current_x:.2f}) vs Target_X({request.x:.2f})")
        
        target_x = request.x
        target_y = request.y
        current_way = request.way
        
        # --- LOGIC ตัดสินใจตามสมการ ---

        # 1. กรณีเป้าหมายอยู่ข้างหน้า (Target X > Current X)
        if target_x > self.current_x:
            self.get_logger().info("✅ Goal is ahead. Sending final_goal immediately.")
            self.publish_final_goal(target_x, target_y, current_way)
            response.update_way = current_way
            return response

        # 2. กรณีเป้าหมายอยู่ข้างหลัง (Target X <= Current X)
        else:
            # สมการสลับ Way: ถ้ามา 'go' ให้เปลี่ยนเป็น 'back', ถ้า 'back' ให้เปลี่ยนเป็น 'go'
            new_way = 'back' if current_way == 'go' else 'go'
            
            self.get_logger().warn(f"🔄 Goal behind! Waiting 2s before rotating to Way: {new_way}")
            
            # หน่วงเวลา 2 วินาที (ใช้ time.sleep ได้เพราะเป็น MultiThreadedExecutor)
            time.sleep(2.0)
            
            # ส่งพิกัดเป้าหมายพร้อม Way ใหม่
            self.publish_final_goal(target_x, target_y, new_way)
            
            # สั่งหมุนตัว
            self.done_event.clear()
            self.send_cmd("left180")
            
            # รอจนกว่าหุ่นจะหมุนเสร็จ (ฟังจาก status_callback)
            self.get_logger().info("⏳ Waiting for rotation to finish...")
            self.done_event.wait() 
            
            response.update_way = new_way
            return response

    def publish_final_goal(self, x, y, way_str):
        # แปลง string way เป็น float เพื่อส่งเข้า Topic เดิม (ถ้าจำเป็น) 
        # สมมติ go=0.0, back=1.0
        way_val = 0.0 if way_str == 'go' else 1.0
        
        final_msg = Float32MultiArray()
        final_msg.data = [float(x), float(y), float(way_val)]
        self.pub_final_goal.publish(final_msg)

    def send_cmd(self, command_str):
        msg = String()
        msg.data = command_str
        self.seq_pub.publish(msg)
        self.waiting_for_status = True 

def main(args=None):
    rclpy.init(args=args)
    node = CheckPositionServer()
    
    # ใช้ MultiThreadedExecutor เพื่อให้สามารถ sleep ในขณะที่รับค่า rtab_pose ได้
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