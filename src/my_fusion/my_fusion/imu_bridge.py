import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray 
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')
        self.subscription = self.create_subscription(
            Float32MultiArray, '/sensors', self.sensor_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_standard', 10)
        self.get_logger().info('IMU Bridge started - Synchronized Time')

    def euler_to_quaternion(self, r, p, y):
        cy, sy = math.cos(y*0.5), math.sin(y*0.5)
        cp, sp = math.cos(p*0.5), math.sin(p*0.5)
        cr, sr = math.cos(r*0.5), math.sin(r*0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def sensor_callback(self, msg):
        if len(msg.data) < 6: return

        imu_msg = Imu()
        # ใช้เวลาปัจจุบัน ห้ามลบ Duration เพื่อให้ EKF จับคู่ข้อมูลได้ทันที
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imubody_link' 

        # Orientation
        imu_msg.orientation = self.euler_to_quaternion(msg.data[0], msg.data[1], msg.data[2])
        
        # Angular Velocity
        imu_msg.angular_velocity.x = msg.data[3]
        imu_msg.angular_velocity.y = msg.data[4]
        imu_msg.angular_velocity.z = msg.data[5]

        # Covariance (เชื่อมั่นในเข็มทิศ/Gyro มากกว่าล้อ)
        imu_msg.orientation_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        imu_msg.linear_acceleration_covariance[0] = -1.0 # ปิด Accel

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()