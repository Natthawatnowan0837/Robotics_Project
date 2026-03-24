#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time

# นำเข้า Service Interface (ตรวจสอบว่าชื่อตรงกับใน srv/ ของคุณ)
from my_command.srv import SequenceCmd

class MoveSequenceNode(Node):
    def __init__(self):
        super().__init__('move_sequence_node')
        
        # ใช้ ReentrantCallbackGroup เพื่อให้ Service, Sensor และ Timer ทำงานขนานกันได้
        self.group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.65)
        self.declare_parameter('move_duration', 1.0)
        
        self.lin_vel = self.get_parameter('linear_speed').value
        self.ang_vel = self.get_parameter('angular_speed').value
        self.duration = self.get_parameter('move_duration').value

        # Variables
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.internal_state = "IDLE" 
        self.is_action_done = False
        self.start_time = 0.0

        # --- [ Pub/Sub ] ---
        # Publisher สำหรับสั่งการล้อ
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber สำหรับรับค่า IMU/Odom จากเซนเซอร์
        self.create_subscription(
            Float32MultiArray, 
            '/sensors', 
            self.sensor_callback, 
            10, 
            callback_group=self.group
        )

        # --- [ Service Server ] ---
        self.srv = self.create_service(
            SequenceCmd, 
            'sequence_cmd_service', 
            self.handle_sequence_cmd,
            callback_group=self.group
        )

        # Control Loop (10Hz) สำหรับส่งความเร็ว
        self.create_timer(0.1, self.control_loop, callback_group=self.group)
        
        self.get_logger().info("=== Move Sequence Service Server Ready ===")

    def sensor_callback(self, msg):
        # รับค่า Yaw (องศา) จากเซนเซอร์ตัวที่ 3
        if len(msg.data) >= 3:
            self.current_yaw = msg.data[2]

    def handle_sequence_cmd(self, request, response):
        """ Callback เมื่อได้รับคำสั่ง Service: จะบล็อกจนกว่าหุ่นยนต์จะขยับเสร็จ """
        cmd = request.state.lower().strip()
        self.get_logger().info(f"📥 Service Request Received: {cmd}")

        # เตรียมตัวเริ่มทำงาน
        self.is_action_done = False
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        if cmd == 'fwd':
            self.internal_state = "FORWARD"
        elif cmd == 'back':
            self.internal_state = "BACKWARD"
        elif cmd == 'left180':
            self.target_yaw = self.current_yaw + 180.0
            self.internal_state = "ROTATING"
        elif cmd == 'right180':
            self.target_yaw = self.current_yaw - 180.0
            self.internal_state = "ROTATING"
        else:
            self.get_logger().error(f"❌ Unknown command: {cmd}")
            response.status = "ERROR" # แก้ให้ตรงกับ .srv (status หรือ state)
            return response

        # --- [ ช่วงรอให้งานเสร็จ ] ---
        # วนลูปเช็คสถานะจนกว่าจะกลับเป็น IDLE (ซึ่งถูกตั้งโดย finish_action)
        while rclpy.ok() and not self.is_action_done:
            time.sleep(0.1)

        # ส่ง Response กลับเมื่อทำงานจบขั้นตอนแล้ว
        response.status = "DONE"
        response.angle = self.current_yaw
        self.get_logger().info(f"📤 Sent Service Response: {cmd} Finished.")
        return response

    def control_loop(self):
        """ ฟังก์ชันคุมการเคลื่อนที่: จะส่งข้อมูลเฉพาะตอนทำงานเท่านั้น """
        if self.internal_state == "IDLE":
            return # สำคัญ: ไม่ Publish ค่า 0 ออกไปกวน Navigation Stack

        msg = Twist()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.internal_state == "FORWARD":
            if (now - self.start_time) < self.duration:
                msg.linear.x = self.lin_vel
            else:
                self.finish_action()

        elif self.internal_state == "BACKWARD":
            if (now - self.start_time) < self.duration:
                msg.linear.x = -self.lin_vel
            else:
                self.finish_action()

        elif self.internal_state == "ROTATING":
            # คำนวณ Error มุม (Normalize -180 to 180)
            error = self.target_yaw - self.current_yaw
            while error > 180: error -= 360
            while error < -180: error += 360
            
            if abs(error) <= 2.5: # ยอมรับความคลาดเคลื่อน 2.5 องศา
                self.finish_action()
            else:
                # หมุนตามทิศทางของ Error
                msg.angular.z = self.ang_vel if error > 0 else -self.ang_vel
        
        # ส่งคำสั่งความเร็วไปยัง Topic /cmd_vel
        self.cmd_pub.publish(msg)

    def finish_action(self):
        """ หยุดหุ่นยนต์และเซ็ตสถานะเป็นพร้อมทำงานขั้นต่อไป """
        # หยุดล้อทันที
        self.cmd_pub.publish(Twist()) 
        self.internal_state = "IDLE"
        
        # รอให้นิ่ง 1 วินาที เพื่อให้กล้องจับภาพ RTAB-Map ได้ชัดเจน
        time.sleep(1.0) 
        
        # ปลดล็อคให้ handle_sequence_cmd ส่ง Response กลับ
        self.is_action_done = True
        self.get_logger().info("✅ Action Finished & Stabilized.")

def main():
    rclpy.init()
    node = MoveSequenceNode()
    
    # ใช้ MultiThreadedExecutor เพื่อให้สามารถรอใน Service ได้โดย Timer ยังทำงานอยู่
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