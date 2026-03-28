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
            Float32MultiArray,
            '/sensors',
            self.sensor_callback,
            10)
            
        self.imu_pub = self.create_publisher(Imu, '/imu/data_standard', 10)
        self.get_logger().info('IMU Bridge Node started with Raw Default Rotation')

    def euler_to_quaternion(self, roll, pitch, yaw):
        # แปลงมุม Euler (Radian) เป็น Quaternion [x, y, z, w]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]

    def sensor_callback(self, msg):
        # ตรวจสอบว่าข้อมูลมาครบ (0-2: Euler, 3-5: Gyro)
        if len(msg.data) < 6:
            return

        # 1. รับค่า Euler ดิบจาก ESP32 (หน่วย Radian)
        # ใช้ลำดับมาตรฐาน [Roll, Pitch, Yaw]
        r_raw = msg.data[0] 
        p_raw = msg.data[1] 
        y_raw = msg.data[2] 

        # 2. แปลงเป็น Quaternion โดยตรง (ไม่มีการหมุน Offset)
        q = self.euler_to_quaternion(r_raw, p_raw, y_raw)

        # 3. สร้าง IMU Message
        imu_msg = Imu()
        now = self.get_clock().now()
        # ใช้เวลาปัจจุบันลบ 0.05s เพื่อให้ TF จับคู่ข้อมูลได้เสถียรขึ้น
        imu_msg.header.stamp = (now - rclpy.duration.Duration(seconds=0.05)).to_msg()
        imu_msg.header.frame_id = 'imubody_link' 

        # ใส่ค่า Orientation (x, y, z, w)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # ใส่ค่า Angular Velocity ดิบ
        imu_msg.angular_velocity.x = msg.data[3]
        imu_msg.angular_velocity.y = msg.data[4]
        imu_msg.angular_velocity.z = msg.data[5]

        # ตั้งค่า Covariance สำหรับ EKF
        imu_msg.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        imu_msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        
        # ปิดการคำนวณ Linear Acceleration ใน EKF
        imu_msg.linear_acceleration_covariance[0] = -1.0 

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU Bridge stopping...')
    finally:
        if rclpy.ok(): # เช็คว่ายังไม่ถูกปิดซ้ำ
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()