#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# นำเข้า Service Interfaces
from my_command.srv import CheckPosition
from my_command.srv import SequenceCmd 

class CheckPositionServer(Node):
    def __init__(self):
        super().__init__('check_position_node')
        
        self.group = ReentrantCallbackGroup()

        # --- [ Publishers ] ---
        self.pub_final_goal = self.create_publisher(Float32MultiArray, 'final_goal', 10)
        
        # --- [ Service Clients ] ---
        # เชื่อมต่อกับโหนดหมุนตัว (RotationControlNode)
        self.rotate_client = self.create_client(
            SequenceCmd, 
            'rotate_service', # เปลี่ยนชื่อให้ตรงกับโหนดหมุน
            callback_group=self.group
        )
        
        # --- [ Subscriptions ] ---
        self.sub_rtab_pose = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/rtabmap/localization_pose', 
            self.rtab_pose_callback, 
            10,
            callback_group=self.group)

        # --- [ Service Server ] ---
        self.srv = self.create_service(
            CheckPosition, 
            'check_position_service', 
            self.handle_check_position,
            callback_group=self.group)

        # --- Variables ---
        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info("🚀 Check Position Node Ready (Integrated with rotate_service).")

    def rtab_pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    async def handle_check_position(self, request, response):
        """ Logic: ถ้าเป้าหมายอยู่ข้างหลัง ให้สั่งหมุน 180 องศา """
        self.get_logger().info(f"🔍 Robot_X({self.current_x:.2f}) | Target_X({request.x:.2f})")
        
        target_x = request.x
        target_y = request.y
        current_way = request.way
        
        # 1. เป้าหมายอยู่ข้างหน้า
        if target_x > self.current_x:
            self.get_logger().info("✅ Goal is ahead. Proceeding...")
            self.publish_final_goal(target_x, target_y, current_way)
            response.update_way = current_way
            return response

        # 2. เป้าหมายอยู่ข้างหลัง (ต้องหมุนตัว)
        else:
            new_way = 'back' if current_way == 'go' else 'go'
            self.get_logger().warn(f"🔄 Goal is behind! Swapping Way to: {new_way}")
            
            # ส่งพิกัดใหม่ไปรอที่ final_goal
            self.publish_final_goal(target_x, target_y, new_way)
            
            # ตรวจสอบว่าโหนดหมุนออนไลน์ไหม
            if not self.rotate_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("❌ Rotate Service NOT AVAILABLE!")
                response.update_way = "ERROR_SERVICE_DOWN"
                return response

            # สร้าง Request สั่งหมุน 180
            req = SequenceCmd.Request()
            req.state = "right180" # คำสั่งหมุน 180 องศา

            self.get_logger().info("⏳ Requesting Rotation (180°)... Waiting for completion.")
            
            try:
                # ใช้ await เพื่อหยุดรอจนกว่าหุ่นจะหมุนเสร็จจริงๆ (Blocking Call)
                future = self.rotate_client.call_async(req)
                result = await future
                
                if result is not None and result.status == "SUCCESS":
                    self.get_logger().info(f"✅ Rotation SUCCESS! Final Angle: {result.angle:.2f}°")
                    response.update_way = new_way
                else:
                    self.get_logger().error("❌ Rotation failed or returned unexpected status.")
                    response.update_way = "ERROR_ROTATION_FAILED"
            except Exception as e:
                self.get_logger().error(f"❌ Exception during rotation: {e}")
                response.update_way = "ERROR_EXCEPTION"
                
            return response

    def publish_final_goal(self, x, y, way_str):
        way_val = 0.0 if way_str == 'go' else 1.0
        final_msg = Float32MultiArray()
        final_msg.data = [float(x), float(y), float(way_val)]
        self.pub_final_goal.publish(final_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CheckPositionServer()
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