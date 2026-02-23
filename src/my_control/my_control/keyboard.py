#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node

# =============================================================
# 1. IMPORT Service ให้ตรงกับ Planner
# =============================================================
from my_command.srv import Sendposition

class KeyboardGoalSender(Node):
    def __init__(self):
        super().__init__('keyboard_goal_sender')
        
        # สร้าง Client เพื่อเชื่อมต่อกับ 'move_robot_service' ของ Planner
        self.cli = self.create_client(Sendposition, 'move_robot_service')
        
        # รอจนกว่า Service ของ Planner จะเปิด
        self.get_logger().info('Waiting for A* Planner service (move_robot_service)...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.get_logger().info('✅ Connected to Planner Service!')

    def send_request(self):
        """รับค่าจากคีย์บอร์ดและส่งไปที่ Planner"""
        try:
            print("\n=========================================")
            print("   SEND GOAL TO A* PLANNER")
            print("=========================================")
            
            # 1. รับค่า Room Name (String)
            room_name = input("Enter Room Name (or 'q' to quit): ")
            if room_name.lower() == 'q':
                return False

            # 2. รับค่า X (Float)
            x_input = input("Enter Target X (meters): ")
            try:
                x = float(x_input)
            except ValueError:
                print("❌ Error: Invalid X value.")
                return True

            # 3. รับค่า Y (Float)
            y_input = input("Enter Target Y (meters): ")
            try:
                y = float(y_input)
            except ValueError:
                print("❌ Error: Invalid Y value.")
                return True

            # 4. รับค่า Floor (Z) (Float/Int)
            floor_input = input("Enter Floor Number (e.g., 1, 2): ")
            try:
                z = float(floor_input)
            except ValueError:
                print("❌ Error: Invalid Floor value.")
                return True

            # สร้าง Request Object
            req = Sendposition.Request()
            req.room_name = room_name
            req.x = x
            req.y = y
            req.z = z # ใน Planner ของคุณใช้ Z แทนชั้น (Floor)

            # ส่ง Request แบบ Async
            self.future = self.cli.call_async(req)
            
            # รอผลลัพธ์
            rclpy.spin_until_future_complete(self, self.future)
            response = self.future.result()

            # แสดงผลลัพธ์ที่ตอบกลับมาจาก Planner
            if response.success:
                self.get_logger().info(f"✅ Success: {response.message}")
            else:
                self.get_logger().warn(f"⚠️ Failed: {response.message}")

        except Exception as e:
            self.get_logger().error(f"Error occurred: {str(e)}")
        
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardGoalSender()
    
    try:
        # วนลูปรับค่าไปเรื่อยๆ จนกว่าผู้ใช้จะกด q
        running = True
        while running:
            running = node.send_request()
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()