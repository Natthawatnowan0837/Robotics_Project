#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped  # Message มาตรฐานที่ RViz ส่งออกมา
from my_command.srv import Sendposition    # Service ของคุณ

class RvizClickToPlanner(Node):
    def __init__(self):
        super().__init__('rviz_click_bridge')

        # 1. สร้าง Subscriber รอรับค่าจากการจิ้มใน Rviz
        # โดยปกติปุ่ม '2D Goal Pose' ใน ROS2 จะส่งเข้า topic '/goal_pose'
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.rviz_callback,
            10)
        
        # 2. สร้าง Client เพื่อส่งต่อไปยัง Planner (เหมือนโค้ดเดิมของคุณ)
        self.cli = self.create_client(Sendposition, 'move_robot_service')

        self.get_logger().info('🖱️  Ready! Click "2D Goal Pose" in Rviz to send target.')

    def rviz_callback(self, msg):
        """ทำงานทันทีเมื่อมีการจิ้มใน Rviz"""
        self.get_logger().info('Received goal from Rviz...')
        
        # ดึงค่า X, Y จากที่จิ้ม
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        
        # กำหนดค่า Z (ชั้น) และ Room Name เอง 
        # (เพราะการจิ้มใน Rviz ไม่บอกเลขชั้นหรือชื่อห้อง)
        default_floor = 1.0
        default_room = "Rviz_Target"

        # เรียกฟังก์ชันส่ง Service
        self.call_planner_service(default_room, target_x, target_y, default_floor)

    def call_planner_service(self, room, x, y, z):
        # ตรวจสอบว่า Service พร้อมไหม
        if not self.cli.service_is_ready():
            self.get_logger().warn('Planner Service is not available!')
            return

        # สร้าง Request
        req = Sendposition.Request()
        req.room_name = room
        req.x = x
        req.y = y
        req.z = z

        # ส่ง Request แบบ Async
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Planner Accepted: {response.message}")
            else:
                self.get_logger().warn(f"⚠️ Planner Rejected: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service Call Failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RvizClickToPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()