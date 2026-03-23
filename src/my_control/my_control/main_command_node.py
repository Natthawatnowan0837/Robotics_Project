#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
import time

class MoveSequenceNode(Node):
    def __init__(self):
        super().__init__('move_sequence_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.65)
        self.declare_parameter('move_duration', 1.5)
        
        self.lin_vel = self.get_parameter('linear_speed').value
        self.ang_vel = self.get_parameter('angular_speed').value
        self.duration = self.get_parameter('move_duration').value

        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.internal_state = "IDLE" 
        self.start_time = 0.0

        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # ส่งสถานะแจ้งเตือนเมื่อทำงานเสร็จ
        self.status_pub = self.create_publisher(String, 'sequence_status', 10)
        
        self.create_subscription(Float32MultiArray, '/sensors', self.sensor_callback, 10)
        self.create_subscription(String, 'sequence_cmd', self.command_callback, 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("=== Move Sequence Node (Done-Signal Mode) Ready ===")

    def sensor_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_yaw = msg.data[2]

    def command_callback(self, msg):
        # รับคำสั่งใหม่ได้ทันที (Override คำสั่งเก่าไปเลย)
        cmd = msg.data.lower().strip()
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f"📥 Executing: {cmd}")

        if cmd == 'fwd':
            self.internal_state = "FORWARD"
        elif cmd == 'back':
            self.internal_state = "BACKWARD"
        elif cmd == 'left180':
            self.target_yaw = self.current_yaw + 180.0
            self.internal_state = "ROTATING"
        elif cmd == 'right180':
            self.target_yaw = self.current_yaw - 180.0
            self.internal_state = "ROTATING"
        else:
            self.get_logger().error(f"❌ Unknown command: {cmd}")

    def control_loop(self):
        msg = Twist()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.internal_state == "FORWARD":
            if (now - self.start_time) < self.duration:
                msg.linear.x = self.lin_vel
            else:
                self.finish_action()

        elif self.internal_state == "BACKWARD":
            if (now - self.start_time) < self.duration:
                msg.linear.x = -self.lin_vel
            else:
                self.finish_action()

        elif self.internal_state == "ROTATING":
            error = self.target_yaw - self.current_yaw
            while error > 180: error -= 360
            while error < -180: error += 360
            
            if abs(error) <= 3.0: 
                self.finish_action()
            else:
                msg.angular.z = self.ang_vel if error > 0 else -self.ang_vel
        
        # ถ้าไม่อยู่ในสถานะเดิน/หมุน (IDLE) cmd_vel จะเป็น 0 โดยอัตโนมัติ
        self.cmd_pub.publish(msg)

    def finish_action(self):
        # 1. หยุดหุ่นยนต์ทันที
        self.cmd_pub.publish(Twist()) 
        self.get_logger().info("✅ Action Finished. Stabilizing camera...")
        
        # 2. รอให้นิ่งเพื่อให้ RTAB-Map ได้ภาพที่ชัดเจน
        # (หมายเหตุ: sleep ใน ROS2 timer อาจทำให้การตอบสนองหน่วงเล็กน้อย 
        # แต่ตอบโจทย์เรื่องการรอผล localize ก่อนส่ง Done)
        time.sleep(1.5) 

        # 3. ส่งสถานะ "DONE" ออกไป เพื่อบอกโหนด AutoSearch ให้ทำขั้นตอนต่อไป
        status_msg = String()
        status_msg.data = "DONE"
        self.status_pub.publish(status_msg)
        
        self.get_logger().info("📢 Sent 'DONE' signal.")
        self.internal_state = "IDLE"

def main():
    rclpy.init()
    node = MoveSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()