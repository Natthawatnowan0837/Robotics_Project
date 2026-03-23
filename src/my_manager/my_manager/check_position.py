#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped 
import threading # เพิ่มเพื่อใช้ Timer

class CheckPosition(Node):
    def __init__(self):
        super().__init__('check_position_node')
        
        # --- Publishers ---
        self.pub_final_goal = self.create_publisher(Float32MultiArray, 'final_goal', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.seq_pub = self.create_publisher(String, 'sequence_cmd', 10)
        
        # --- Subscriptions ---
        self.sub_action = self.create_subscription(String, 'action', self.action_callback, 10)
        self.sub_rtab_pose = self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose', self.rtab_pose_callback, 10)
        self.sub_goal = self.create_subscription(Float32MultiArray, 'pub_goal', self.goal_callback, 10)
        self.sub_seq_status = self.create_subscription(String, 'sequence_status', self.status_callback, 10)

        # --- Variables ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_way = 0.0
        self.waiting_for_status = False
        
        # --- Timer Setup ---
        self.startup_timer = None
        self.startup_delay = 3.0

        self.get_logger().info("🚀 Check Position Node Ready. (Auto-Reset & 3s Delay Active)")

    def rtab_pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def goal_callback(self, msg):
        if len(msg.data) >= 2:
            self.goal_x = msg.data[0]
            self.goal_y = msg.data[1]
            self.goal_way = msg.data[2] if len(msg.data) >= 3 else 0.0
            
            # เมื่อได้เป้าหมายใหม่ ให้ยกเลิก Timer เก่าที่อาจจะค้างอยู่
            if self.startup_timer is not None:
                self.startup_timer.cancel()
            
            self.waiting_for_status = False
            self.get_logger().info(f"📥 New Goal Received: [{self.goal_x}, {self.goal_y}] - Resetting states.")

    def status_callback(self, msg):
        if msg.data == "1" or msg.data.upper() == "DONE":
            if self.waiting_for_status:
                self.get_logger().info("✅ Rotation Done. Reporting back to Manager...")
                self.waiting_for_status = False
                self.report_done()

    def action_callback(self, msg):
        command = msg.data.lower().strip()
        
        # หากได้รับคำสั่ง 'position' ให้เริ่มการนับถอยหลัง 3 วินาทีก่อนทำงาน
        if command == "position":
            # ยกเลิก Timer เก่าก่อนเริ่มอันใหม่ (Reset)
            if self.startup_timer is not None:
                self.startup_timer.cancel()
            
            self.waiting_for_status = False
            self.get_logger().info(f"⏳ Received 'position'. Waiting {self.startup_delay}s before comparing...")
            
            # เริ่มนับถอยหลัง 3 วินาที แล้วไปเรียก execute_position_check
            self.startup_timer = threading.Timer(self.startup_delay, self.execute_position_check)
            self.startup_timer.start()
            
        else:
            # หากได้รับคำสั่งอื่น ให้ Reset Timer และสถานะ
            if self.startup_timer is not None:
                self.startup_timer.cancel()
            self.waiting_for_status = False

    def execute_position_check(self):
        """ ฟังก์ชันนี้จะทำงานหลังจาก Delay ครบ 3 วินาที """
        self.get_logger().info(f"🔍 [DELAY DONE] Comparing: Local_X({self.current_x:.2f}) vs Goal_X({self.goal_x:.2f})")

        # 1. กรณี Goal อยู่ข้างหน้า
        if self.goal_x > self.current_x:
            self.get_logger().info("✅ Goal is ahead. Publishing final_goal...")
            self.publish_final_goal(self.goal_way)
            self.report_done()

        # 2. กรณี Goal อยู่ข้างหลัง
        else:
            new_way = 0.0 if self.goal_way == 1.0 else 1.0
            self.get_logger().warn(f"🔄 Goal behind! Toggling Way: {self.goal_way} -> {new_way}")
            
            self.publish_final_goal(new_way)
            
            self.get_logger().warn("📤 Sending 'left180' command...")
            self.send_cmd("left180")

    def publish_final_goal(self, way_value):
        final_msg = Float32MultiArray()
        final_msg.data = [float(self.goal_x), float(self.goal_y), float(way_value)]
        self.pub_final_goal.publish(final_msg)

    def send_cmd(self, command_str):
        msg = String()
        msg.data = command_str
        self.seq_pub.publish(msg)
        self.waiting_for_status = True 

    def report_done(self):
        status_msg = String()
        status_msg.data = "position,done"
        self.status_pub.publish(status_msg)
        self.get_logger().info(f"🏁 Published: {status_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CheckPosition()

    # --- ส่วนที่ต้องแก้: เปลี่ยนจาก spin ปกติ เป็น MultiThreadedExecutor ---
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # สั่งให้ทำงานแบบหลาย Thread
        executor.spin() 
    except KeyboardInterrupt:
        node.get_logger().info("🛑 ปิดระบบ Check Position")
    finally:
        if node.startup_timer is not None:
            node.startup_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()