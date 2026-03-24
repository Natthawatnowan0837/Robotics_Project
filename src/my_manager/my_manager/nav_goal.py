#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from my_command.srv import Nav2 
import time

class Nav2ServiceServer(Node):
    def __init__(self):
        super().__init__('nav2_service_node')
        
        # ใช้ ReentrantCallbackGroup เพื่อให้ Service และ Feedback ทำงานขนานกันได้
        self.group = ReentrantCallbackGroup()

        # --- [ Initialize Nav2 BasicNavigator ] ---
        self.get_logger().info('⏳ Initializing Nav2 BasicNavigator...')
        self.nav = BasicNavigator()

        # 1. บังคับให้ Nav2 Nodes เป็น Active (สำคัญมาก)
        self.get_logger().info('🔧 Activating Nav2 Lifecycle Nodes...')
        self.nav.lifecycleStartup() 
        
        # 2. รอจนกว่าระบบจะพร้อม (ใช้ bt_navigator สำหรับ RTAB-Map)
        self.nav.waitUntilNav2Active(localizer='bt_navigator')
        
        self.get_logger().info('✅ Nav2 Stack is fully Active and Ready.')

        # --- [ Create Service Server ] ---
        self.srv = self.create_service(
            Nav2, 
            'nav2_service', 
            self.handle_nav2_request,
            callback_group=self.group
        )

        self.get_logger().info('🚀 Nav2 Service Server is online.')

    def handle_nav2_request(self, request, response):
            try:
                self.get_logger().info(f"📥 Received Target: X={request.x}, Y={request.y}")
                
                # เคลียร์ Task เก่าทิ้งก่อน
                if not self.nav.isTaskComplete():
                    self.nav.cancelTask()

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = float(request.x)
                goal_pose.pose.position.y = float(request.y)
                goal_pose.pose.orientation.w = 1.0

                # สั่งเดินทันที (GoToPose จะหา Path เองโดยอัตโนมัติ)
                self.nav.goToPose(goal_pose)

                # เช็คสถานะการเดิน
                while not self.nav.isTaskComplete():
                    # ตรวจสอบว่า Nav2 ยังมีชีวิตอยู่ไหมในขณะเดิน
                    if not self.nav.isNav2Active():
                        self.get_logger().error("💀 Nav2 Stack went Offline!")
                        break
                    
                    feedback = self.nav.getFeedback()
                    if feedback:
                        self.get_logger().info(f'🛰️ Distance: {feedback.distance_remaining:.2f} m', throttle_duration_sec=2.0)
                    
                    time.sleep(0.5) # พักบ้างเพื่อลดภาระ CPU

                result = self.nav.getResult()
                response.success = (result == TaskResult.SUCCEEDED)
                return response

            except Exception as e:
                self.get_logger().error(f"❌ Crash Avoided: {e}")
                response.success = False
                return response

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ServiceServer()
    
    # ใช้ MultiThreadedExecutor เพื่อรองรับ Service และ Callback ที่ซ้อนกัน
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Nav2 Service Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()