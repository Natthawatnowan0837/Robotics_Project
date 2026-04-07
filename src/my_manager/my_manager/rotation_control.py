#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time
import math
import re

from my_command.srv import SequenceCmd

class MoveSequenceNode(Node):
    def __init__(self):
        super().__init__('move_sequence_node')
        
        self.group = ReentrantCallbackGroup()

        # --- [ Parameters ] ---
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 55.0)
        self.declare_parameter('move_duration', 3.0)
        self.declare_parameter('tolerance', 2.0)
        
        self.lin_vel = self.get_parameter('linear_speed').value
        self.ang_vel_deg = self.get_parameter('angular_speed').value
        self.default_duration = self.get_parameter('move_duration').value
        self.tolerance = self.get_parameter('tolerance').value

        # Variables
        self.current_yaw_deg = 0.0
        self.target_yaw_deg = 0.0
        self.internal_state = "IDLE" 
        self.is_action_done = False
        self.start_time = 0.0
        self.active_duration = 0.0

        # --- [ Pub/Sub ] ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_rotation', 10)
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
            'rotate_service', 
            self.handle_sequence_cmd,
            callback_group=self.group
        )

        # Control Loop (10Hz)
        self.create_timer(0.1, self.control_loop, callback_group=self.group)
        
        self.get_logger().info("🚀 Move Sequence Server Ready! (With STOP support)")

    def sensor_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_yaw_deg = math.degrees(msg.data[2])
            self.get_logger().info(f"📡 Yaw: {self.current_yaw_deg:.2f}°", throttle_duration_sec=1.0)

    def handle_sequence_cmd(self, request, response):
        cmd = request.state.lower().strip()
        self.get_logger().info(f"📥 Received Command: {cmd}")

        # --- [ 1. ระบบ STOP ] ---
        if cmd == 'stop':
            self.internal_state = "IDLE"
            self.cmd_pub.publish(Twist())
            self.is_action_done = True
            response.status = "STOPPED"
            return response

        self.is_action_done = False
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # --- [ 2. แยกข้อความและตัวเลขด้วย Regex ] ---
        # ค้นหาตัวเลข (รวมทศนิยม) ที่อยู่ใน String
        match = re.search(r"[-+]?\d*\.\d+|\d+", cmd)
        value = float(match.group()) if match else None

        # --- [ 3. ตรวจสอบเงื่อนไขคำสั่ง ] ---
        if cmd.startswith('fwd') or cmd.startswith('back'):
            # ถ้าไม่มีตัวเลขตามหลัง ให้ใช้ค่า default (3.0s) ถ้ามีให้ใช้ตาม input
            self.active_duration = value if value is not None else self.default_duration
            self.internal_state = "FORWARD" if cmd.startswith('fwd') else "BACKWARD"
            self.get_logger().info(f"🚚 Moving {self.internal_state} for {self.active_duration}s")

        elif 'left' in cmd or 'right' in cmd:
            # ถ้าไม่มีตัวเลขตามหลัง ให้ใช้ค่า 82.0 องศา (ตามที่คุณตั้งไว้)
            angle = value if value is not None else 82.0
            
            if 'left' in cmd:
                self.target_yaw_deg = self.current_yaw_deg + angle
            else:
                self.target_yaw_deg = self.current_yaw_deg - angle
                
            self.internal_state = "ROTATING"
            self.get_logger().info(f"🔄 Rotating {angle} deg -> Target: {self.target_yaw_deg:.2f}°")

        else:
            self.get_logger().error(f"❌ Unknown format: {cmd}")
            response.status = "ERROR"
            return response

        # Loop รอจนกว่า Action จะเสร็จ
        while rclpy.ok() and not self.is_action_done:
            time.sleep(0.05)

        response.status = "DONE"
        response.angle = self.current_yaw_deg
        return response

    def control_loop(self):
        # ถ้าสถานะเป็น IDLE ไม่ต้องทำอะไรและไม่ต้อง Publish
        if self.internal_state == "IDLE":
            return

        msg = Twist()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.internal_state == "FORWARD":
            if (now - self.start_time) < self.active_duration:
                msg.linear.x = self.lin_vel
            else:
                self.finish_action()

        elif self.internal_state == "BACKWARD":
            if (now - self.start_time) < self.active_duration:
                msg.linear.x = -self.lin_vel
            else:
                self.finish_action()

        elif self.internal_state == "ROTATING":
            error = self.target_yaw_deg - self.current_yaw_deg
            while error > 180: error -= 360
            while error < -180: error += 360
            
            if abs(error) <= self.tolerance:
                self.finish_action()
                return

            speed_multiplier = 1.0 if abs(error) > 15 else 0.4
            out_vel_deg = (self.ang_vel_deg * speed_multiplier) if error > 0 else -(self.ang_vel_deg * speed_multiplier)
            msg.angular.z = math.radians(out_vel_deg)

        self.cmd_pub.publish(msg)

    def finish_action(self):
        """ หยุดการเคลื่อนที่และกลับสู่สถานะพร้อมรับคำสั่งใหม่ """
        self.cmd_pub.publish(Twist()) 
        self.internal_state = "IDLE"
        self.get_logger().info("🎯 Action Finished.")
        time.sleep(0.5) 
        self.is_action_done = True

def main():
    rclpy.init()
    node = MoveSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # ก่อนปิด Node ให้มั่นใจว่าหุ่นยนต์หยุดนิ่ง
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()