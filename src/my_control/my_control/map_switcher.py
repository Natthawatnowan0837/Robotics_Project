#!/usr/bin/env python3
import os
import sys
import threading
import math
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory
from functools import partial

class MapSwitcher(Node):
    def __init__(self):
        super().__init__('map_switcher')
        
        # -------------------------------------------------------------
        # [NEW] 1. การตั้งค่าพิกัดเริ่มต้นของแต่ละชั้น (หน้าลิฟต์/บันได)
        # แก้ไขค่า x, y, yaw (องศา) ให้ตรงกับแผนที่จริงของคุณ
        # -------------------------------------------------------------
        self.floor_config = {
            1: {'name': 'floor1', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            2: {'name': 'floor2', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            3: {'name': 'floor3', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            4: {'name': 'floor4', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        }

        # 2. สร้าง Client สำหรับเปลี่ยนแผนที่
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        
        # [NEW] 3. สร้าง Publisher สำหรับตั้งค่า Initial Pose
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 4. รอให้ Map Server พร้อม
        self.get_logger().info('Waiting for /map_server/load_map service...')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Still waiting for map server...')

        self.pkg_share = get_package_share_directory('my_command')
        
        self.get_logger().info('=========================================')
        self.get_logger().info('   MAP SWITCHER + AUTO POSE READY')
        self.get_logger().info('   Please enter floor number (1-4)')
        self.get_logger().info('   or "q" to quit')
        self.get_logger().info('=========================================')

        # เริ่ม Thread รับ Input
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Enter Command [1-4]: ")
                if user_input.lower() == 'q':
                    self.get_logger().info("Quitting...")
                    rclpy.shutdown()
                    break
                
                if user_input in ['1', '2', '3', '4']:
                    self.process_command(int(user_input))
                else:
                    print("Invalid input! Please enter 1-4.")
            except EOFError:
                break

    def process_command(self, floor_num):
        # ดึงข้อมูลจาก Config
        floor_data = self.floor_config.get(floor_num)
        subfolder = floor_data['name']
        filename = "rtabmap.yaml" # หรือ map.yaml แล้วแต่คุณตั้ง

        # สร้าง Path เต็ม
        map_path = os.path.join(self.pkg_share, 'maps', subfolder, filename)
        
        if not os.path.exists(map_path):
             self.get_logger().error(f'File not found: {map_path}')
             return

        # เรียกฟังก์ชันเปลี่ยนแผนที่ (ส่งเลขชั้นไปด้วย)
        self.change_map(map_path, floor_num)

    def change_map(self, map_path, floor_num):
        self.get_logger().info(f'Requesting Switch -> Floor {floor_num}...')
        
        request = LoadMap.Request()
        request.map_url = map_path
        
        # ส่ง Request และแนบ floor_num ไปกับ callback ด้วย partial
        future = self.map_client.call_async(request)
        future.add_done_callback(partial(self.response_callback, floor_num=floor_num))

    def response_callback(self, future, floor_num):
        try:
            response = future.result()
            if response.result == 0: # สำเร็จ
                self.get_logger().info(f'>>> Map Floor {floor_num} Loaded! Setting Initial Pose... <<<')
                
                # [NEW] เรียกฟังก์ชันตั้งจุดเริ่มต้น
                self.set_initial_pose(floor_num)
                
                print("Enter Command [1-4]: ", end='', flush=True)
            else:
                self.get_logger().error(f'>>> Failed to switch map (Code: {response.result}) <<<')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def set_initial_pose(self, floor_num):
        """ ฟังก์ชันส่งค่าตำแหน่งเริ่มต้นให้ Nav2/AMCL """
        data = self.floor_config[floor_num]
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # ตั้งตำแหน่ง X, Y
        msg.pose.pose.position.x = float(data['x'])
        msg.pose.pose.position.y = float(data['y'])
        msg.pose.pose.position.z = 0.0
        
        # แปลง Yaw (องศา) เป็น Quaternion (x,y,z,w)
        # สูตรลัดสำหรับหมุนรอบแกน Z
        yaw_rad = math.radians(data['yaw'])
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        # กำหนดความเชื่อมั่น (Covariance) เล็กน้อยเพื่อให้ AMCL รู้ว่าเรามั่นใจ
        # 0.25 คือค่า Variance เริ่มต้นมาตรฐาน
        msg.pose.covariance[0] = 0.25  # X
        msg.pose.covariance[7] = 0.25  # Y
        msg.pose.covariance[35] = 0.068 # Yaw (rotation)

        self.init_pose_pub.publish(msg)
        self.get_logger().info(f"Initial Pose Set to: x={data['x']}, y={data['y']}")

def main(args=None):
    rclpy.init(args=args)
    node = MapSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()