#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from my_command.srv import Nav2 # Interface: float32 x, float32 y -> bool success
import time

class Nav2ServiceServer(Node):
    def __init__(self):
        super().__init__('nav2_service_node')
        
        self.group = ReentrantCallbackGroup()

        # --- [ Initialize Nav2 Navigator ] ---
        self.get_logger().info('⏳ Initializing Nav2 BasicNavigator...')
        self.nav = BasicNavigator()
        
        # 🚩 แก้ไขตรงนี้: สั่งให้รอแบบไม่ต้องเช็ค AMCL (ใช้ bt_navigator แทน)
        # หรือใช้ self.nav.waitUntilNav2Active(localizer='bt_navigator')
        self.nav.waitUntilNav2Active(localizer='bt_navigator')
        
        self.get_logger().info('✅ Nav2 Lifecycle Nodes are Active.')
        # --- [ Service Server ] ---
        self.srv = self.create_service(
            Nav2, 
            'nav2_service', 
            self.handle_nav2_request,
            callback_group=self.group
        )

        self.get_logger().info('🚀 Nav2 Service Server is Ready.')

    def handle_nav2_request(self, request, response):
        # เก็บค่าจาก request เข้าตัวแปร (Logger จะโชว์ตรงนี้)
        target_x = float(request.x)
        target_y = float(request.y)

        # --- [ ส่วนที่แสดงผล req.x และ req.y ] ---
        self.get_logger().info("-" * 30)
        self.get_logger().info(f"📥 New Nav2 Request Received:")
        self.get_logger().info(f"📍 Target X: {target_x:.2f}")
        self.get_logger().info(f"📍 Target Y: {target_y:.2f}")
        self.get_logger().info("-" * 30)

        # 1. สร้าง Pose สำหรับเป้าหมาย
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_x
        goal_pose.pose.position.y = target_y
        goal_pose.pose.orientation.w = 1.0 

        # 2. สั่ง Nav2 ให้เริ่มเดิน
        self.get_logger().info("🗺️ Sending Goal to Nav2 Stack and Waiting for Plan...")
        self.nav.goToPose(goal_pose)

        # 3. วนลูปติดตามสถานะ
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback:
                # แสดงระยะที่เหลือ และค่าความเร็ว (ถ้าต้องการ)
                self.get_logger().info(
                    f'🛰️ Distance to goal: {feedback.distance_remaining:.2f} m | '
                    f'Time elapsed: {feedback.navigation_time.sec} s', 
                    throttle_duration_sec=2.0
                )
            time.sleep(0.1)

        # 4. ตรวจสอบผลลัพธ์
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('🏁 SUCCESS: Robot reached the goal!')
            response.success = True
        else:
            self.get_logger().error(f'❌ FAILED: Navigation ended with result code: {result}')
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ServiceServer()
    
    # ใช้ MultiThreadedExecutor เพื่อให้ Callback ของ Feedback และ Service ทำงานร่วมกันได้
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()