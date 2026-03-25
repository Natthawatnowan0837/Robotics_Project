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
        self.group = ReentrantCallbackGroup()

        self.get_logger().info('⏳ Connecting to Nav2 BasicNavigator...')
        self.nav = BasicNavigator()
        
        # รอให้ระบบ Active
        self.nav.waitUntilNav2Active(localizer='bt_navigator')
        self.get_logger().info('✅ Nav2 Stack is ONLINE.')

        self.srv = self.create_service(
            Nav2, 
            'nav2_service', 
            self.handle_nav2_request,
            callback_group=self.group
        )

    def handle_nav2_request(self, request, response):
        try:
            self.get_logger().info(f"📥 Received Request: X={request.x}, Y={request.y}")
            
            # ป้องกันการส่งพิกัดศูนย์
            if request.x == 0.0 and request.y == 0.0:
                self.get_logger().error("🚫 Received 0,0 target. Rejecting.")
                response.success = False
                return response

            # ยกเลิกงานเก่าถ้ามี
            if not self.nav.isTaskComplete():
                self.nav.cancelTask()
                time.sleep(0.5)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(request.x)
            goal_pose.pose.position.y = float(request.y)
            goal_pose.pose.orientation.w = 1.0

            self.nav.goToPose(goal_pose)

            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                if feedback:
                    self.get_logger().info(
                        f'🛰️ Remaining: {feedback.distance_remaining:.2f} m', 
                        throttle_duration_sec=5.0
                    )
                time.sleep(1.0)

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("🏁 Task Succeeded!")
                response.success = True
            else:
                self.get_logger().error(f"❌ Task Failed with result code: {result}")
                response.success = False

            return response

        except Exception as e:
            self.get_logger().error(f"❌ Service Error: {e}")
            response.success = False
            return response

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ServiceServer()
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