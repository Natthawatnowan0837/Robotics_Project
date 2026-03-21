import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import os
from ament_index_python.packages import get_package_share_directory

class MapSelector(Node):
    def __init__(self):
        super().__init__('map_selector_node')
        
        # ชื่อ Package ของคุณ
        self.package_name = 'your_package_name' # <--- เปลี่ยนเป็นชื่อ package จริง
        
        # Subscribe ไปที่ topic 'select_map' 
        # รับค่าเป็น [ชั้น, สถานะ] เช่น [1, 1]
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'select_map',
            self.map_callback,
            10)
            
        self.get_logger().info('Map Selector Node has been started.')

    def map_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn('Invalid message format. Expected [floor, status]')
            return

        floor_num = msg.data[0]
        status = msg.data[1] # 1 สำหรับ go, 0 สำหรับ back

        # แปลงสถานะเป็นชื่อไฟล์
        file_name = 'go.db' if status == 1 else 'back.db'
        
        # ค้นหาพาธของ package ในส่วน share
        try:
            package_share_path = get_package_share_directory(self.package_name)
            
            # สร้าง Path ไปยังไฟล์: share/package_name/maps/floorX/file.db
            map_path = os.path.join(
                package_share_path, 
                'maps', 
                f'floor{floor_num}', 
                file_name
            )

            if os.path.exists(map_path):
                self.get_logger().info(f'Selected Map Path: {map_path}')
                # ตรงนี้คุณสามารถนำ map_path ไปใช้งานต่อ เช่น ส่งให้ Navigation หรือ Load DB
            else:
                self.get_logger().error(f'File not found: {map_path}')
                
        except Exception as e:
            self.get_logger().error(f'Error finding package: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()