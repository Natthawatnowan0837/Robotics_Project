import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class EKFTestNode(Node):
    def __init__(self):
        super().__init__('ekf_test_node')
        
        # รับค่าจาก Encoder ดิบ
        self.create_subscription(Odometry, '/odom', self.odom_raw_callback, 10)
        # รับค่าจาก EKF (ที่ Fusion แล้ว)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        
        self.raw_yaw = 0.0
        self.ekf_yaw = 0.0

    def quaternion_to_euler(self, q):
        # แปลง Quaternion เป็น Yaw (Z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_raw_callback(self, msg):
        self.raw_yaw = math.degrees(self.quaternion_to_euler(msg.pose.pose.orientation))

    def ekf_callback(self, msg):
        self.ekf_yaw = math.degrees(self.quaternion_to_euler(msg.pose.pose.orientation))
        self.display_status()

    def display_status(self):
        # พิมพ์ค่าออกมาเทียบกัน
        print(f"--- FUSION TEST ---")
        print(f"RAW Odom Yaw (Wheel): {self.raw_yaw:.2f} deg")
        print(f"EKF Filtered Yaw (IMU Fusion): {self.ekf_yaw:.2f} deg")
        print(f"Difference: {abs(self.raw_yaw - self.ekf_yaw):.2f} deg")
        
        # เช็คสถานะ
        if abs(self.raw_yaw - self.ekf_yaw) > 0.1:
            print("Status: Fusion is ACTIVE (IMU is correcting Wheel)")
        else:
            print("Status: Sensors are in sync")
        print("-------------------")

def main():
    rclpy.init()
    node = EKFTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()