import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class MotorToOdom(Node):
    def __init__(self):
        super().__init__('motor_to_odom_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motors',
            self.motor_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # สถานะหุ่นยนต์
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.get_logger().info('Motor to Odom Node started (TF disabled for EKF)')

    def motor_callback(self, msg):
        if len(msg.data) < 6:
            return

        # ดึงค่าจาก ESP32 (ปรับเครื่องหมายตามจริง)
        d_center = -msg.data[4]  
        d_theta = -msg.data[5]   
        v_linear = msg.data[2]
        v_angular = msg.data[3]

        # 1. Dead Reckoning (Mid-point Integration)
        avg_th = self.th + (d_theta / 2.0)
        self.x += d_center * math.cos(avg_th)
        self.y += d_center * math.sin(avg_th)
        self.th += d_theta

        # 2. สร้าง Message และจัดการเวลา (หัวใจสำคัญ)
        current_time = self.get_clock().now().to_msg()
        q = self.euler_to_quaternion(0.0, 0.0, self.th)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint' # เชื่อมกับพื้นตาม URDF

        # Pose (X, Y, Theta)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        
        # Covariance: เชื่อ X,Y แต่ไม่เชื่อ Yaw (ให้ EKF ไปเอาจาก IMU แทน)
        odom.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.8] # Yaw เชื่อน้อยหน่อย

        # Twist (Velocity)
        odom.twist.twist.linear.x = float(v_linear)
        odom.twist.twist.angular.z = float(v_angular)

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
        cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
        cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = MotorToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()