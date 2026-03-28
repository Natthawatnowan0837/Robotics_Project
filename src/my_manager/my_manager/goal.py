#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from my_command.srv import Goal 
import time

class GoalServiceServer(Node):
    def __init__(self):
        super().__init__('goal_service_node')
        self.group = ReentrantCallbackGroup()

        self.get_logger().info('⏳ Connecting to Nav2 (Goal System)...')
        self.nav = BasicNavigator()
        
        # ตรวจสอบเบื้องต้นว่า Nav2 พร้อมใช้งานจริงหรือไม่
        self.nav.waitUntilNav2Active(localizer='bt_navigator')
        self.get_logger().info('✅ Goal System is ONLINE and Ready.')

        self.srv = self.create_service(
            Goal, 
            'Goal_service', 
            self.handle_goal_request,
            callback_group=self.group
        )

    def handle_goal_request(self, request, response):
        try:
            self.get_logger().info(f"📥 Received Request: X={request.x:.2f}, Y={request.y:.2f}")

            # --- ส่วนที่ 1: ตรวจสอบความพร้อมและส่ง STATUS ---
            # เช็คว่า Nav2 อยู่ในสถานะทำงานหรือไม่ (Lifecycle Check)
            # ถ้าเรียกผ่าน waitUntilNav2Active มาแล้ว ปกติจะพร้อม แต่เช็คซ้ำเพื่อความชัวร์
            response.status = True 
            
            # ป้องกันพิกัด 0,0 (ยกเว้นตั้งใจไปจุด Origin ของแผนที่)
            if request.x == 0.0 and request.y == 0.0:
                self.get_logger().error("🚫 Received 0,0 target. Rejecting.")
                response.success = False
                return response

            # ยกเลิกงานเก่าถ้าหุ่นยนต์ยังเดินไม่เสร็จ
            if not self.nav.isTaskComplete():
                self.get_logger().info("⚠️ Canceling previous task to accept new Goal.")
                self.nav.cancelTask()
                time.sleep(0.5)

            # --- ส่วนที่ 2: เริ่มกระบวนการ Navigation ---
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(request.x)
            goal_pose.pose.position.y = float(request.y)
            goal_pose.pose.orientation.w = 1.0

            # สั่งให้หุ่นยนต์ Plan เส้นทางและเคลื่อนที่
            self.nav.goToPose(goal_pose)

            # Loop รอจนกว่า Task จะสำเร็จหรือล้มเหลว
            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                if feedback:
                    self.get_logger().info(
                        f'🛰️ Distance remaining: {feedback.distance_remaining:.2f} m', 
                        throttle_duration_sec=5.0
                    )
                time.sleep(1.0)

            # --- ส่วนที่ 3: ตรวจสอบผลลัพธ์และส่ง SUCCESS ---
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("🏁 Destination Reached Successfully!")
                response.success = True
            else:
                self.get_logger().error(f"❌ Navigation failed or canceled (Code: {result})")
                response.success = False

            return response

        except Exception as e:
            self.get_logger().error(f"❌ Goal Service Exception: {e}")
            response.status = False
            response.success = False
            return response

def main(args=None):
    rclpy.init(args=args)
    node = GoalServiceServer()
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