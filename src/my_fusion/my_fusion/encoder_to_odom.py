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
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ตัวแปรสถานะของหุ่นยนต์ (เริ่มต้นที่ 0,0,0)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.get_logger().info('Motor to Odom Node has started with TF Broadcaster')

    def motor_callback(self, msg):
        # ตรวจสอบว่าข้อมูลมาครบ (อย่างน้อย 6 ช่อง: 0-5)
        if len(msg.data) < 6:
            return

        # ดึงค่า Delta จาก ESP32 (ปรับเครื่องหมายตามทิศทางหุ่นจริง)
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
        # ในระนาบ 2D เราหมุนรอบแกน Z เท่านั้น (yaw = self.th)
        q_msg = self.euler_to_quaternion(0.0, 0.0, self.th)

        # --- 3. Publish TF Transform (สำคัญมากสำหรับ RViz) ---
        # t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q_msg

        # ส่ง TF ออกไปยังระบบ
        # self.tf_broadcaster.sendTransform(t)

        # --- 4. Publish Odometry Message ---
        odom = Odometry()
        odom.header.stamp = t.header.stamp # ใช้เวลาเดียวกับ TF
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # ตำแหน่ง (Pose)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q_msg
        
        # Pose Covariance (ความเชื่อมั่น)
        odom.pose.covariance = [0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  999.0, 0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  999.0, 0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  999.0, 0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  0.05]

        # ความเร็ว (Twist)
        odom.twist.twist.linear.x = float(v_linear)
        odom.twist.twist.angular.z = float(v_angular)
        
        # Twist Covariance
        odom.twist.covariance = odom.pose.covariance # ใช้ค่าเดียวกันเบื้องต้น

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # สูตรแปลง Euler เป็น Quaternion สำหรับ geometry_msgs
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        q.w = cr * cp * cy + sr * sp * sy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = MotorToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()