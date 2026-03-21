import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math

class MotorToOdom(Node):
    def __init__(self):
        super().__init__('motor_to_odom_node')

        # Subscribe ค่าจาก ESP32
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motors',
            self.motor_callback,
            10)

        # Publisher สำหรับ Odom และ TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ตัวแปรสถานะของหุ่นยนต์ (เริ่มต้นที่ 0,0,0)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def motor_callback(self, msg):
        # ตรวจสอบว่าข้อมูลมาครบ (อย่างน้อย 6 ช่อง: 0-5)
        if len(msg.data) < 6:
            return

        # ดึงค่า Delta จาก ESP32
        d_center = -msg.data[4]  # ระยะทางที่เคลื่อนได้ใน loop นี้ (เมตร)
        d_theta = -msg.data[5]   # มุมที่เปลี่ยนไปใน loop นี้ (เรเดียน)
        
        # ดึงค่าความเร็ว (สำหรับการแสดงผลใน Odom message)
        v_linear = msg.data[2]
        v_angular = msg.data[3]

        # --- 1. คำนวณสะสมตำแหน่ง (Dead Reckoning) ---
        # ใช้ Mid-point Integration เพื่อความแม่นยำ
        avg_th = self.th + (d_theta / 2.0)
        self.x += d_center * math.cos(avg_th)
        self.y += d_center * math.sin(avg_th)
        self.th += d_theta

        # --- 2. สร้าง Quaternion จากมุม Euler (theta) ---
        q = self.euler_to_quaternion(0, 0, self.th)

        # --- 3. Publish TF Transform (odom -> base_link) ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
    

        # --- 4. Publish Odometry Message ---
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # ตำแหน่ง (Pose)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        # ความเร็ว (Twist)
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = MotorToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()