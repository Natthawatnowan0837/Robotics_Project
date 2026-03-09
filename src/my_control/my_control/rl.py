import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class RLMonitor(Node):
    def __init__(self):
        super().__init__('rl_monitor')
        
        # --- แก้ไขชื่อ Topic ให้ตรงกับ ros2 topic list ของคุณ ---
        self.create_subscription(Float32MultiArray, '/sensors', self.sensor_callback, 10)
        self.create_subscription(Float32MultiArray, '/balance', self.pwm_callback, 10)
        
        self.current_sensors = [0.0] * 7 
        self.current_pwm = 0.0
        
        self.get_logger().info('--- กำลังเชื่อมต่อกับ Topic: /sensors และ /balance ---')

    def sensor_callback(self, msg):
        # เก็บข้อมูลจาก /sensors
        if len(msg.data) >= 7:
            self.current_sensors = msg.data

    def pwm_callback(self, msg):
        # เก็บข้อมูลจาก /balance (สมมติ PWM อยู่ index 1)
        if len(msg.data) >= 2:
            self.current_pwm = msg.data[1]
        
        # ดึงค่ามาแสดงผล
        ang_b = self.current_sensors[1]
        ang_p = self.current_sensors[3]
        hall  = self.current_sensors[4]
        gyr_b = self.current_sensors[5]
        gyr_p = self.current_sensors[6]
        pwm   = self.current_pwm

        # แสดงผลบรรทัดเดียว (ใช้ \r เพื่อทับบรรทัดเดิม)
        print(f"BodyY:{ang_b:6.2f} | PlatY:{ang_p:6.2f} | Hall:{int(hall)} | GB:{gyr_b:6.2f} | GP:{gyr_p:6.2f} | PWM:{pwm:6.2f}", end='\r')

def main(args=None):
    rclpy.init(args=args)
    node = RLMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStop Monitor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()