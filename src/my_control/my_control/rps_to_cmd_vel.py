import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math

class RpsToCmdVel(Node):
    def __init__(self):
        super().__init__('rps_to_cmd_vel_node')

        # --- 1. กำหนดค่าคงที่ของหุ่นยนต์ (ปรับแก้ตรงนี้) ---
        self.wheel_radius = 0.033       # รัศมีล้อ (เช่น 3.3 ซม. = 0.033 ม.)
        self.wheel_separation = 0.16    # ระยะห่างล้อ (เช่น 16 ซม. = 0.16 ม.)

        # --- 2. สร้าง Subscriber และ Publisher ---
        # รับค่า [RPS_Left, RPS_Right]
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_rps_arrey',
            self.rps_callback,
            10)
        
        # ส่งค่าความเร็วเชิงเส้น/เชิงมุมมาตรฐาน
        self.publisher_ = self.create_publisher(Twist, '/odom_velocity', 10)

        self.get_logger().info('RPS to CmdVel Node has started!')

    def rps_callback(self, msg):
        # ตรวจสอบว่าข้อมูลมาครบ 2 ล้อไหม
        if len(msg.data) < 2:
            self.get_logger().warn('Received RPS array with less than 2 elements!')
            return

        # ดึงค่า RPS จาก Array (สมมติ index 0 = ซ้าย, 1 = ขวา)
        rps_left = msg.data[0]
        rps_right = msg.data[1]

        # --- 3. สูตรคำนวณ Kinematics ---
        # แปลง RPS เป็น Linear Velocity ของแต่ละล้อ (v = r * omega_rad)
        # omega_rad = RPS * 2 * pi
        v_left = rps_left * (2 * math.pi * self.wheel_radius)
        v_right = rps_right * (2 * math.pi * self.wheel_radius)

        # คำนวณความเร็วรวมของหุ่นยนต์
        linear_v = (v_right + v_left) / 2.0
        angular_v = (v_right - v_left) / self.wheel_separation

        # --- 4. สร้าง Message และส่งออก ---
        twist = Twist()
        twist.linear.x = linear_v
        twist.angular.z = angular_v

        self.publisher_.publish(twist)

        # (Optional) Log ดูค่าที่คำนวณได้
        # self.get_logger().info(f'Linear: {linear_v:.2f} m/s, Angular: {angular_v:.2f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = RpsToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()