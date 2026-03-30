#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from my_command.srv import Controller # หรือ Service ที่ภาใช้เรียกปีน/ควบคุม
import math
import time

class HardCodeSequenceNode(Node):
    def __init__(self):
        super().__init__('hard_code_sequence_node')
        self.group = ReentrantCallbackGroup()

        # --- [ Parameters ] ---
        self.kp_yaw = 0.5
        self.max_ang_vel = 0.6
        self.linear_speed = 0.2  # ความเร็วเดินหน้า
        self.tolerance = 2.0      # ระยะยอมรับองศา (องศา)

        # Variables
        self.current_yaw_deg = 0.0
        self.is_active = False

        # --- [ Sub/Pub/Srv ] ---
        # รับค่า Odom เพื่อเช็คองศา
        self.create_subscription(
            Odometry, '/odom/filtered', self.odom_callback, 10, callback_group=self.group)
        
        # ส่งคำสั่งควบคุมล้อ
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # สร้าง Service เพื่อให้ State Manager เรียกใช้งาน
        self.srv = self.create_service(
            Controller, 'hard_code_service', self.handle_service, callback_group=self.group)

        self.get_logger().info("🚀 Hard Code Node Ready: Rotate 90 -> Fwd 4s")

    def odom_callback(self, msg):
        # แปลง Quaternion เป็น Euler (Degree)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    async def handle_service(self, request, response):
        if not request.active:
            self.is_active = False
            self.stop_robot()
            response.success = False
            return response

        self.get_logger().info("🎬 Starting Sequence: Rotate Left 90...")
        self.is_active = True

        # --- [ STEP 1: หมุนซ้าย 90 องศา ] ---
        start_yaw = self.current_yaw_deg
        target_yaw = start_yaw + 90.0
        
        # จัดการเรื่องมุมเกิน 180/-180 (Wrap Angle)
        if target_yaw > 180: target_yaw -= 360

        while self.is_active and rclpy.ok():
            error = target_yaw - self.current_yaw_deg
            # Shortest path
            if error > 180: error -= 360
            elif error < -180: error += 360

            if abs(error) < self.tolerance:
                self.stop_robot()
                break
            
            msg = Twist()
            msg.angular.z = self.max_ang_vel if error > 0 else -self.max_ang_vel
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

        self.get_logger().info("✅ Rotation Finished. Moving Forward for 4s...")

        # --- [ STEP 2: เดินหน้าตรง 4 วินาที ] ---
        if self.is_active:
            msg = Twist()
            msg.linear.x = self.linear_speed
            
            start_time = time.time()
            while time.time() - start_time < 4.0 and self.is_active:
                self.cmd_pub.publish(msg)
                time.sleep(0.1)
            
            self.stop_robot()

        self.get_logger().info("🏁 Sequence Completed!")
        self.is_active = False
        response.success = True
        return response

def main():
    rclpy.init()
    node = HardCodeSequenceNode()
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