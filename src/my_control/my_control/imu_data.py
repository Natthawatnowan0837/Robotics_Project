import rclpy
from rclpy.node import Node
# เปลี่ยนจาก Float64 เป็น Float32
from std_msgs.msg import Float32MultiArray 
from sensor_msgs.msg import Imu
import math

class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')
        
        # แก้ไขชนิดข้อความเป็น Float32MultiArray
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/sensors',
            self.sensor_callback,
            10)
            
        self.imu_pub = self.create_publisher(Imu, '/imu/data_standard', 10)
        self.get_logger().info('IMU Bridge Node has started with Float32 support')

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0] * 4
        q[0] = sr * cp * cy - cr * sp * sy # x
        q[1] = cr * sp * cy + sr * cp * sy # y
        q[2] = cr * cp * sy - sr * sp * cy # z
        q[3] = cr * cp * cy + sr * sp * sy # w
        return q

    def sensor_callback(self, msg):
        # ตรวจสอบจำนวนข้อมูล (Index 0-7 ตามที่คุณระบุไว้ก่อนหน้า)
        if len(msg.data) < 8:
            return

        # แปลงหน่วย Degree เป็น Radian
        # [0]=Roll, [1]=Pitch, [2]=Yaw
        roll  = math.radians(msg.data[0])
        pitch = math.radians(msg.data[1])
        yaw   = math.radians(msg.data[2])
        
        # Gyro (Angular Velocity): [3]=Rate X, [4]=Rate Y, [5]=Rate Z
        gyro_x = math.radians(msg.data[3])
        gyro_y = math.radians(msg.data[4])
        gyro_z = math.radians(msg.data[5])

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        # ใช้ base_link เป็นจุดอ้างอิงหลักของหุ่นยนต์ [cite: 66]
        imu_msg.header.frame_id = 'base_link' 

        # Orientation
        q = self.euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # Angular Velocity
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        # Covariance สำหรับ EKF (0.01 คือค่าความเชื่อมั่นระดับปานกลาง)
        imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        # ใส่ -1 ในตำแหน่งแรกเพื่อบอกว่าไม่มีข้อมูล Linear Acceleration
        imu_msg.linear_acceleration_covariance[0] = -1.0 

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()