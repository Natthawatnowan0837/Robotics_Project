import rclpy
from rclpy.node import Node
# นำเข้า Message type (ปรับเปลี่ยนได้ตามที่ใช้งานจริง)
from std_msgs.msg import Float32MultiArray

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber_node')
        
        # สร้าง Subscriber รับข้อมูลจาก topic "/sensors"
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensors',
            self.listener_callback,
            10)
        
        self.get_logger().info('Sensor Subscriber Node เริ่มทำงานแล้ว...')
        self.get_logger().info('รอรับข้อมูลจาก /sensors และจะแสดงข้อมูลตัวที่ 10 (Index 9)')

    def listener_callback(self, msg):
        # ตรวจสอบก่อนว่าข้อมูลที่ได้รับมีอย่างน้อย 10 ตัวหรือไม่
        # เนื่องจาก Python เริ่มนับ index ที่ 0 ดังนั้นตัวที่ 10 คือ index 9
        if len(msg.data) >= 10:
            target_value = msg.data[9]
            self.get_logger().info(f'ได้รับข้อมูล: ตัวที่ 10 (Index 9) มีค่าเท่ากับ: {target_value:.4f}')
        else:
            self.get_logger().warn(f'ข้อมูลไม่ครบ: ได้รับมาเพียง {len(msg.data)} ตัว (ต้องการอย่างน้อย 10 ตัว)')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()