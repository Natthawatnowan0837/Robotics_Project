import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class MapNavigator(Node):
    def __init__(self):
        super().__init__('map_navigator')
        
        # 1. สร้างตัวแปร Navigator
        self.nav = BasicNavigator()

        # 2. รอจนกว่าระบบ Nav2 จะพร้อม (ข้าม AMCL ไปเช็ค bt_navigator)
        self.get_logger().info('กำลังรอระบบ Navigation... (Waiting for bt_navigator)')
        self.nav.waitUntilNav2Active(localizer='bt_navigator')
        self.get_logger().info('ระบบพร้อมใช้งาน! แสตนบายรอรับเป้าหมายที่ Topic: /goal')

        # 3. สร้าง Subscriber รอรับพิกัดจาก Topic ชื่อ "goal"
        # ใช้ Message type เป็น PoseStamped เพื่อความยืดหยุ่น
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal',
            self.goal_callback,
            10)

    def goal_callback(self, msg):
        """ ฟังก์ชันนี้จะทำงานทันทีเมื่อมีข้อมูลส่งเข้ามาที่ Topic /goal """
        x = msg.pose.position.x
        y = msg.pose.position.y
        w = msg.pose.orientation.w if msg.pose.orientation.w != 0.0 else 1.0

        self.get_logger().info(f'ได้รับพิกัดใหม่: x={x:.2f}, y={y:.2f} กำลังเริ่มเดินทาง...')
        
        # เรียกใช้ฟังก์ชันส่งเป้าหมาย
        self.send_nav_goal(x, y, w)

    def send_nav_goal(self, x, y, w):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = w

        self.nav.goToPose(goal_pose)

        # วนลูปเช็คสถานะ
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            # คุณสามารถใส่สถานะ Feedback ตรงนี้ได้
            pass

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('ถึงจุดหมายสำเร็จ!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('ภารกิจถูกยกเลิก')
        elif result == TaskResult.FAILED:
            self.get_logger().error('ภารกิจล้มเหลว!')

def main(args=None):
    rclpy.init(args=args)
    node = MapNavigator()
    
    # ใช้ spin เพื่อให้ Node ทำงานค้างไว้รอรับ Subscriber
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()