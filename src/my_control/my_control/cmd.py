import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.subscription = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop) # วิ่งที่ 10Hz
        self.current_path = None

    def plan_callback(self, msg):
        self.current_path = msg

    def control_loop(self):
        if self.current_path is None or len(self.current_path.poses) == 0:
            return

        # 1. เลือกจุดเป้าหมาย (ในที่นี้เลือกจุดที่ 5 บนเส้นทางเพื่อเป็น Lookahead)
        target_index = min(len(self.current_path.poses) - 1, 5)
        target_pose = self.current_path.poses[target_index].pose

        # 2. คำนวณความต่างของตำแหน่ง (สมมติหุ่นยนต์อยู่ที่ 0,0 ใน frame ของแผนที่)
        # *ในระบบจริงต้องใช้ TF เพื่อหาตำแหน่งหุ่นยนต์เทียบกับแผนที่ก่อน*
        dx = target_pose.position.x
        dy = target_pose.position.y
        
        angle_to_target = math.atan2(dy, dx)

        # 3. สร้างคำสั่งความเร็ว
        msg = Twist()
        msg.linear.x = 0.15 # วิ่งช้าๆ 
        msg.angular.z = 1.5 * angle_to_target # ปรับจูนค่า P gain ตรงนี้

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    rclpy.shutdown()