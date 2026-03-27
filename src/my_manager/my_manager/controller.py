#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from pynput import keyboard
import time

# Import service interface
from my_command.srv import Controller 

class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('keyboard_controller_node')
        
        # --- [ ระบบ State Control ] ---
        self.is_active = False
        self.mission_completed = False # ตัวแปรเช็คว่ากด Enter หรือยัง
        
        self.srv = self.create_service(
            Controller, 
            'controller_service', 
            self.handle_controller
        )

        # --- [ Configuration ] ---
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.linear_max = self.get_parameter('linear_speed').value
        self.angular_max = self.get_parameter('angular_speed').value

        self.current_body_angle = 0.0
        self.target_angle = None
        self.is_auto_turning = False
        self.pressed_keys = set()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/sensors', self.angle_callback, 10)

        # Keyboard Listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🎮 Controller Ready. Press ENTER to finish mission after Active.")

    def handle_controller(self, request, response):
        if request.active:
            self.get_logger().info("📥 [SERVICE] Controller Active: Control the robot...")
            self.is_active = True
            self.mission_completed = False
            
            # 🛑 วนลูปค้างไว้ใน Service จนกว่า mission_completed จะเป็น True (จากการกด Enter)
            while rclpy.ok() and not self.mission_completed:
                if not self.is_active: break # เผื่อกรณีโดนแทรกแซง
                time.sleep(0.1)
            
            self.is_active = False # ปิดการทำงานเมื่อจบ
            response.success = True
            self.get_logger().info("📤 [SERVICE] Mission Success: Enter pressed, returning to Manager.")
        else:
            self.is_active = False
            response.success = False
        return response

    def on_press(self, key):
        if not self.is_active: return
        
        try:
            # --- [ เช็คการกดปุ่ม Enter ] ---
            if key == keyboard.Key.enter:
                self.get_logger().info("🎯 [KEYBOARD] Enter pressed! Ending mission...")
                self.mission_completed = True
                return

            if hasattr(key, 'char'):
                key_char = key.char.lower()
                self.pressed_keys.add(key_char)
                
                if key_char == 'l' and not self.is_auto_turning:
                    self.start_auto_turn(-180.0)
                elif key_char == 'j' and not self.is_auto_turning:
                    self.start_auto_turn(180.0)
        except Exception as e:
            self.get_logger().error(f"Error on_press: {e}")

    def on_release(self, key):
        try:
            if hasattr(key, 'char'):
                char = key.char.lower()
                if char in self.pressed_keys:
                    self.pressed_keys.remove(char)
        except: pass

    def angle_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_body_angle = msg.data[2]

    def start_auto_turn(self, angle_offset):
        self.target_angle = self.current_body_angle + angle_offset
        self.is_auto_turning = True

    def control_loop(self):
        if not self.is_active or self.mission_completed:
            self.cmd_vel_pub.publish(Twist())
            return

        twist = Twist()
        if self.is_auto_turning:
            error = self.target_angle - self.current_body_angle
            if abs(error) < 3.0:
                self.is_auto_turning = False
                self.get_logger().info("✅ Turn Complete.")
            else:
                twist.angular.z = self.angular_max if error > 0 else -self.angular_max
        else:
            if 'w' in self.pressed_keys: twist.linear.x = self.linear_max
            if 's' in self.pressed_keys: twist.linear.x = -self.linear_max
            if 'a' in self.pressed_keys: twist.angular.z = self.angular_max
            if 'd' in self.pressed_keys: twist.angular.z = -self.angular_max

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    # ใช้ MultiThreadedExecutor เพื่อให้ Service ไม่ไปบล็อก Keyboard Listener
    from rclpy.executors import MultiThreadedExecutor
    node = KeyboardControllerNode()
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