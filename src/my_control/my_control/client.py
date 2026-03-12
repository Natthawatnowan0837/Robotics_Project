import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from my_command.srv import Sendposition 
# from my_command.srv import CheckFloor

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')

        self.srv = self.create_service(Sendposition, 'move_robot_service', self.goal_callback)
        # self.srv = self.create_service(CheckFloor, 'move_robot_service', self.check_callback)
        
    
        self.goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        
        self.get_logger().info('Service Server & Publisher พร้อมทำงานแล้ว...')

    # def goal_callback(self, request, response):
    #     msg = PoseStamped()
    #     msg.pose.position.x = request.z

    #     self.goal_pub.publish(msg)
    #     response.success = True
    #     response.message = f"Robot is moving to {request.room_name}"
    #     return

    def goal_callback(self, request, response):
        # --- Logger Info ---
        self.get_logger().info('>>> [ได้รับคำสั่งใหม่] <<<')
        self.get_logger().info(f'ชื่อสถานที่: {request.room_name} | พิกัด X: {request.x:.2f} Y: {request.y:.2f} Z: {request.z}')

        # --- สร้าง Message เพื่อ Publish ไปยัง /goal ---
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = request.x
        msg.pose.position.y = request.y
        msg.pose.position.z = 0.0 # Nav2 ใช้ Z ของพิกัดปกติเป็น 0
        msg.pose.orientation.w = 1.0 # หน้าตรงเสมอ

        # Publish ข้อมูลออกไป
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Published goal for {request.room_name} to /goal topic')
        
        # --- ตอบกลับ Service ---
        response.success = True
        response.message = f"Robot is moving to {request.room_name}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    rclpy.shutdown()