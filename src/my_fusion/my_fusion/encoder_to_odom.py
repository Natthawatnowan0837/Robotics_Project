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

        # Publisher สำหรับ Odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # --- ข้อแนะนำสำหรับคุณ "ภา" ---
        # หากใช้ EKF (robot_localization) ให้คอมเมนต์ tf_broadcaster ไว้แบบเดิม
        # แต่ถ้าต้องการทดสอบโหนดนี้เดี่ยวๆ (ไม่มี EKF) ให้เอาคอมเมนต์ออกครับ
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.get_logger().info('Motor to Odom Node has started (TF remains disabled for EKF)')

    def motor_callback(self, msg):
        if len(msg.data) < 6:
            return

        # ดึงค่า Delta (ระวังเครื่องหมาย +/- ตามการเข็นจริง)
        d_center = -msg.data[4]  
        d_theta = -msg.data[5]   
        
        v_linear = msg.data[2]
        v_angular = msg.data[3]

        # --- 1. Dead Reckoning ---
        avg_th = self.th + (d_theta / 2.0)
        self.x += d_center * math.cos(avg_th)
        self.y += d_center * math.sin(avg_th)
        self.th += d_theta

        # --- 2. สร้าง Quaternion ---
        q_msg = self.euler_to_quaternion(0.0, 0.0, self.th)
        current_time = self.get_clock().now().to_msg()

        # --- 3. สร้าง Odometry Message ---
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # ตำแหน่ง (Pose)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q_msg
        
        # --- 4. ปรับ Covariance (สำคัญสำหรับการปีนบันได) ---
        # เราเชื่อ X, Y จากล้อ แต่เราจะให้ EKF เชื่อ Yaw จาก IMU มากกว่า
        odom.pose.covariance = [
            0.005, 0.0,   0.0,   0.0,   0.0,   0.0,  # x
            0.0,   0.005, 0.0,   0.0,   0.0,   0.0,  # y
            0.0,   0.0,   999.0, 0.0,   0.0,   0.0,  # z
            0.0,   0.0,   0.0,   999.0, 0.0,   0.0,  # roll
            0.0,   0.0,   0.0,   0.0,   999.0, 0.0,  # pitch
            0.0,   0.0,   0.0,   0.0,   0.0,   0.5   # yaw (ค่าสูงขึ้นเพื่อให้เชื่อ IMU มากกว่า)
        ]

        # ความเร็ว (Twist)
        odom.twist.twist.linear.x = float(v_linear)
        odom.twist.twist.angular.z = float(v_angular)
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

        # --- 5. Publish TF (ใช้สำหรับทดสอบแบบไม่มี EKF เท่านั้น) ---
        # t = TransformStamped()
        # t.header.stamp = current_time
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation = q_msg
        # self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
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
        node.get_logger().info('Node stopping...')
    finally:
        # ป้องกันการ shutdown ซ้ำซ้อน
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()