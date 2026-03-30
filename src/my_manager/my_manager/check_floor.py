#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from my_command.srv import CheckFloor 

class Check_floor(Node):
    def __init__(self):
        super().__init__('floor_manager_node')
        
        self.sub_sensors = self.create_subscription(
            Float32MultiArray, 'sensors', self.sensors_callback, 10)

        self.srv = self.create_service(
            CheckFloor, 'check_floor_service', self.check_floor_callback)

        # --- [ ส่วนที่แก้ไข ] ---
        # 1. ตั้งค่าเริ่มต้นเป็น 2.0
        self.current_floor = 2.0  
        self.call_count = 0
        
        self.get_logger().info(f"🚀 Floor Manager Ready: Start at {self.current_floor}, Limit at 3.0")

    def sensors_callback(self, msg):
        """ Log ค่าเซนเซอร์ """
        if len(msg.data) >= 11:
            p = msg.data[9]
            self.get_logger().info(f"📊 Live P: {p:.2f} | Current State: Floor {self.current_floor}", throttle_duration_sec=10.0)

    def check_floor_callback(self, request, response):
        self.call_count += 1
        target_f = request.floor

        # --- [ Logic: เริ่มที่ 2 บวกเพิ่มได้แต่ไม่เกิน 3 ] ---
        # ถ้าเป็นการเรียกครั้งที่ 2 เป็นต้นไป ให้พยายามบวกชั้น
        if self.call_count > 1:
            # ใช้ min() เพื่อล็อคค่าไม่ให้เกิน 3.0
            self.current_floor = min(3.0, self.current_floor + 1.0)
            
            if self.current_floor == 3.0:
                self.get_logger().info("✅ Reached Limit: Floor is now capped at 3.0")
            else:
                self.get_logger().info(f"⬆️ Incrementing Floor: Now at {self.current_floor}")
        else:
            # ครั้งแรกที่เรียก จะยังคงเป็น 2.0 ตามค่าเริ่มต้น
            self.get_logger().info(f"1️⃣ First Request: Staying at Initial Floor {self.current_floor}")

        # ส่งค่าปัจจุบันกลับไปใน Response
        current_f = self.current_floor
        response.current_floor = current_f

        # โลจิกเปรียบเทียบ Up / Down / same_floor
        if target_f == current_f:
            response.status = "same_floor"
        elif target_f > current_f:
            response.status = "Up"
        elif target_f < current_f:
            response.status = "Down"
        
        self.get_logger().info(f"🎯 Result: {response.status} (Target:{target_f}, Current:{current_f})")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Check_floor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()