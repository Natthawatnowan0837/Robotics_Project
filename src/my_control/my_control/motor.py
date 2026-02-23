#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy

class GoToPath2(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # QoS เพื่อให้รับข้อมูลได้ชัวร์ขึ้น
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Publisher / Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        
        # แก้ชื่อ Topic ให้ตรงกับ Planner (/planned_path)
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.path = []
        self.current_target = 0
        self.reached = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Motor Controller Started. Waiting for /planned_path")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.yaw = yaw

    def path_callback(self, msg: Path):
        # รับ Path ใหม่ -> รีเซ็ตเป้าหมายเป็นจุดแรก
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_target = 0
        self.reached = False
        # self.get_logger().info(f"New path received: {len(self.path)} points")

    def control_loop(self):
        if not self.path or self.reached:
            return

        try:
            goal_x, goal_y = self.path[self.current_target]
        except IndexError:
            self.stop_robot()
            return

        inc_x = goal_x - self.x
        inc_y = goal_y - self.y
        distance = math.hypot(inc_x, inc_y)
        angle_to_goal = math.atan2(inc_y, inc_x)
        angle_diff = math.atan2(math.sin(angle_to_goal - self.yaw), math.cos(angle_to_goal - self.yaw))

        cmd = Twist()

        # เช็คว่าถึงจุด Waypoint หรือยัง (ระยะ 0.15 เมตร)
        if distance < 0.15:
            self.current_target += 1
            if self.current_target >= len(self.path):
                self.get_logger().info("Goal Reached!")
                self.stop_robot()
                self.reached = True
                self.path = [] # เคลียร์ Path ทิ้ง
            return

        # P-Controller
        MAX_LIN = 0.35
        MAX_ANG = 1.2

        if abs(angle_diff) > 0.4: # ถ้าหันผิดทางเยอะ ให้หมุนอยู่กับที่
            cmd.linear.x = 0.0
            cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, 1.5 * angle_diff))
        else: # ถ้าหันถูกทางแล้ว ให้วิ่งพร้อมเลี้ยว
            cmd.linear.x = max(0.0, min(MAX_LIN, distance)) # ลดความเร็วเมื่อใกล้ถึง
            cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, 2.0 * angle_diff))

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd) # ส่ง 0.0 เพื่อหยุด
        self.get_logger().info("Robot Stopped.")
        # ลบส่วน Reset Odom ออก เพราะผิดหลักการ

def main(args=None):
    rclpy.init(args=args)
    node = GoToPath2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()