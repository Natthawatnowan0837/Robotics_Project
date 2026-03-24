#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# นำเข้า Service Interfaces (ตรวจสอบชื่อแพ็คเกจและชื่อไฟล์ .srv)
from my_command.srv import CheckPosition
from my_command.srv import SequenceCmd 

class CheckPositionServer(Node):
    def __init__(self):
        super().__init__('check_position_node')
        
        # ใช้ Reentrant เพื่อให้ callback ของ Service และ Subscriber ทำงานขนานกันได้
        self.group = ReentrantCallbackGroup()

        # --- [ Publishers ] ---
        self.pub_final_goal = self.create_publisher(Float32MultiArray, 'final_goal', 10)
        
        # --- [ Service Clients ] ---
        # สร้าง Client เพื่อไปเรียกใช้โหนดลูก (MoveSequenceNode)
        self.sequence_client = self.create_client(
            SequenceCmd, 
            'sequence_cmd_service', 
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
        # ตัวนี้จะรอรับคำสั่งตรวจสอบตำแหน่งจากโหนดอื่น (เช่น Main Control)
        self.srv = self.create_service(
            CheckPosition, 
            'check_position_service', 
            self.handle_check_position,
            callback_group=self.group)

        # --- Variables ---
        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info("🚀 Check Position Service (Async-Service Mode) Ready.")

    def rtab_pose_callback(self, msg):
        """ อัปเดตตำแหน่งปัจจุบันของหุ่นยนต์จาก RTAB-Map """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    async def handle_check_position(self, request, response):
        """ Logic ตัดสินใจ: ถ้าเป้าหมายอยู่ข้างหลัง ให้สั่งหมุนตัวก่อน """
        self.get_logger().info(f"🔍 Checking: Robot_X({self.current_x:.2f}) vs Target_X({request.x:.2f})")
        
        target_x = request.x
        target_y = request.y
        current_way = request.way
        
        # 1. กรณีเป้าหมายอยู่ข้างหน้า (เดินต่อได้เลย)
        if target_x > self.current_x:
            self.get_logger().info("✅ Goal is ahead. Sending final_goal.")
            self.publish_final_goal(target_x, target_y, current_way)
            response.update_way = current_way
            return response

        # 2. กรณีเป้าหมายอยู่ข้างหลัง (ต้องสั่งหมุน 180 องศา)
        else:
            new_way = 'back' if current_way == 'go' else 'go'
            self.get_logger().warn(f"🔄 Goal behind! Waiting for stabilize and rotate to Way: {new_way}")
            
            # หยุดนิ่งสักพักก่อนหมุน
            time.sleep(1.0)
            
            # ส่งพิกัดใหม่ไปที่ final_goal
            self.publish_final_goal(target_x, target_y, new_way)
            
            # ตรวจสอบว่าโหนดลูก (MoveSequenceNode) ออนไลน์อยู่ไหม
            if not self.sequence_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("❌ Sequence Service not available!")
                response.update_way = "ERROR_SERVICE_DOWN"
                return response

            # สร้าง Request สำหรับสั่งหมุนตัว
            req = SequenceCmd.Request()
            req.active = "true"
            req.state = "left180"

            self.get_logger().info("⏳ Calling Rotation Service... Waiting for 'DONE'")
            
            try:
                # ใช้ await เพื่อหยุดรอจนกว่าโหนดลูกจะส่ง Response กลับมา
                result = await self.sequence_client.call_async(req)
                
                # ตรวจสอบผลลัพธ์ (เช็คค่า .status ตามไฟล์ .srv)
                if result is not None and result.status == "DONE":
                    self.get_logger().info(f"✅ Rotation finished. New Angle: {result.angle:.2f}")
                    response.update_way = new_way
                else:
                    self.get_logger().error("❌ Rotation failed or returned unexpected status.")
                    response.update_way = "ERROR_ROTATION_FAILED"
            except Exception as e:
                self.get_logger().error(f"❌ Exception during service call: {e}")
                response.update_way = "ERROR_EXCEPTION"
                
            return response

    def publish_final_goal(self, x, y, way_str):
        """ ส่งข้อมูลไปยัง Topic final_goal """
        # แปลง string เป็นค่า float (0.0=go, 1.0=back)
        way_val = 0.0 if way_str == 'go' else 1.0
        final_msg = Float32MultiArray()
        final_msg.data = [float(x), float(y), float(way_val)]
        self.pub_final_goal.publish(final_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CheckPositionServer()
    
    # ต้องใช้ MultiThreadedExecutor เพราะมีการใช้ await ภายใน Service Handler
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