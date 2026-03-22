#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import subprocess
import time
import os
import signal

class MapNavigator(Node):
    def __init__(self):
        super().__init__('map_navigator')
        
        # --- 1. ตั้งค่า Path ของไฟล์ Config ---
        # แนะนำให้ใช้ path ที่ชี้ไปยังไฟล์ใน install หลังจาก colcon build แล้ว
        self.params_file = '/home/noone/Robotics_Project/src/my_control/config/nav2_params.yaml'
        
        # --- 2. สั่ง Launch Nav2 ใน Background ---
        self.nav2_process = None
        self.launch_nav2_stack()

        # --- 3. เริ่มต้น BasicNavigator ---
        self.nav = BasicNavigator()

        # --- 4. รอจนกว่าระบบจะพร้อม (ใช้ bt_navigator แทน amcl เพราะเราทำ SLAM) ---
        self.get_logger().info('⏳ กำลังรอระบบ Navigation... (Waiting for bt_navigator)')
        
        # รอจนกว่าระบบจะ Active (Timeout 60 วินาที)
        self.nav.waitUntilNav2Active(localizer='bt_navigator')
        
        self.get_logger().info('✅ ระบบพร้อมใช้งาน! รอรับเป้าหมายที่ Topic: /goal')

        # --- 5. สร้าง Subscriber ---
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal',
            self.goal_callback,
            10)

    def launch_nav2_stack(self):
        """ ฟังก์ชันสำหรับสั่งรันคำสั่ง Launch ผ่าน subprocess """
        if not os.path.exists(self.params_file):
            self.get_logger().error(f'❌ ไม่พบไฟล์ params ที่: {self.params_file}')
            return

        cmd = [
            'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
            'use_sim_time:=false',
            f'params_file:={self.params_file}',
            'use_amcl:=false',
            'map:=/rtabmap/map'
        ]

        self.get_logger().info(f'🚀 กำลังรันคำสั่ง: {" ".join(cmd)}')
        
        try:
            # ใช้ start_new_session เพื่อให้กระบวนการลูกไม่ตายเมื่อ Node หลักมีปัญหาชั่วคราว
            self.nav2_process = subprocess.Popen(
                cmd, 
                stdout=subprocess.DEVNULL, # ปิด Log ของ Nav2 ใน Terminal นี้ (เพื่อความสะอาด)
                stderr=subprocess.STDOUT
            )
            time.sleep(5) # ให้เวลา Nav2 ตั้งตัวนิดนึง
        except Exception as e:
            self.get_logger().error(f'❌ ไม่สามารถ Launch Nav2 ได้: {str(e)}')

    def goal_callback(self, msg):
        """ เมื่อได้รับพิกัดจาก /goal """
        x = msg.pose.position.x
        y = msg.pose.position.y
        w = msg.pose.orientation.w if msg.pose.orientation.w != 0.0 else 1.0

        self.get_logger().info(f'📍 รับเป้าหมายใหม่: x={x:.2f}, y={y:.2f}')
        self.send_nav_goal(x, y, w)

    def send_nav_goal(self, x, y, w):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = w

        self.nav.goToPose(goal_pose)

        # วนลูปเช็คสถานะแบบ Non-blocking (พิมพ์ Feedback ทุก 3 วินาที)
        i = 0
        while not self.nav.isTaskComplete():
            i += 1
            feedback = self.nav.getFeedback()
            if feedback and i % 15 == 0:
                self.get_logger().info(f'🛰️ ระยะห่างจากเป้าหมาย: {feedback.distance_remaining:.2f} เมตร')
            time.sleep(0.2)

        # ตรวจสอบผลลัพธ์หลังเสร็จสิ้น
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('🏁 ถึงจุดหมายสำเร็จ! เยี่ยมมาก พี')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('⚠️ ภารกิจถูกยกเลิก')
        elif result == TaskResult.FAILED:
            self.get_logger().error('❌ ภารกิจล้มเหลว! ตรวจสอบสิ่งกีดขวางรอบตัวหุ่นยนต์')

    def destroy_node(self):
        """ ปิด Process Nav2 เมื่อปิด Node """
        if self.nav2_process:
            self.get_logger().info('🛑 กำลังปิดระบบ Nav2...')
            os.kill(self.nav2_process.pid, signal.SIGINT)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MapNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 กำลังปิดโปรแกรม...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()