import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import Info

class Check_localize(Node):
    def __init__(self):
        super().__init__('loc_status_checker')
        
        # Subscribe ไปที่ rtabmap/info
        self.subscription = self.create_subscription(
            Info,
            '/rtabmap/info',
            self.info_callback,
            10)
        
        self.get_logger().info('🟢 Localization Status Checker Started!')
        print("-" * 50)

    def info_callback(self, msg):
        # 1. เช็ค Loop Closure ID (ถ้า > 0 คือเจอจุดที่คุ้นเคย)
        loop_id = msg.loop_closure_id
        
        # 2. เช็คค่าความเชื่อมั่น (Hypothesis Value) 
        # ปกติจะอยู่ที่ Index สุดท้ายของ posterior_values หรือหาจาก stats
        # ในที่นี้เราดึงจาก loop_closure_id เป็นหลักจะชัวร์สุด
        
        if loop_id > 0:
            status = "✅ LOCALIZED"
            color_code = "\033[92m"  # สีเขียว
        else:
            status = "❌ LOST / SEARCHING"
            color_code = "\033[91m"  # สีแดง
            
        reset_color = "\033[0m"

        # พิมพ์สถานะออกแบบบรรทัดเดียว (Overwrite) เพื่อไม่ให้รก Terminal
        print(f"{color_code}{status}{reset_color} | Loop ID: {loop_id:<4} | Ref ID: {msg.ref_id:<4}", end='\r')

def main(args=None):
    rclpy.init(args=args)
    node = Check_localize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 ปิดระบบเช็คสถานะ")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()