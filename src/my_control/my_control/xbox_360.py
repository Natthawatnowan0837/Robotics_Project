#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # เปลี่ยนจาก Vector3 เป็น Twist
import pygame

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        
        # เปลี่ยน Message Type เป็น Twist
        self.move_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Multiplier จะกลายเป็นตัวคูณความเร็วสูงสุด (Max Speed)
        self.multiplier = 1.0  
        
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller found!")
            return
            
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(f"Xbox Ready (Twist Mode) | Max Speed Multiplier: {self.multiplier}")

        self.timer = self.create_timer(0.1, self.controller_loop)

    def controller_loop(self):
            pygame.event.pump()
            msg = Twist() 

            # --- Multiplier Logic ---
            if self.joy.get_button(0): # A
                self.multiplier = max(0.0, self.multiplier - 0.1)
                self.get_logger().info(f"Max Speed: {self.multiplier:.1f}")
            elif self.joy.get_button(3): # X
                self.multiplier = min(2.0, self.multiplier + 0.1)
                self.get_logger().info(f"Max Speed: {self.multiplier:.1f}")
            elif self.joy.get_button(1): # B
                self.multiplier = 0.0
                self.get_logger().info("Speed Reset to 0.0")

            # --- Get D-pad state ---
            dpad = self.joy.get_hat(0)
            
            # กำหนดค่าพื้นฐานเป็น 0
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            if dpad != (0, 0):
                if dpad == (0, 1):     # Forward
                    msg.linear.x = 0.3 * self.multiplier
                elif dpad == (0, -1):  # Backward
                    msg.linear.x = -0.3 * self.multiplier
                elif dpad == (-1, 0):  # Turn Left
                    msg.angular.z = 1.0 * self.multiplier
                elif dpad == (1, 0):   # Turn Right
                    msg.angular.z = -1.0 * self.multiplier
                
                self.get_logger().info(f"Moving: L={msg.linear.x:.2f}, A={msg.angular.z:.2f}")
            
            # ส่งข้อมูลเสมอ (เพื่อให้ Failsafe ใน ESP32 ไม่ทำงาน)
            self.move_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()