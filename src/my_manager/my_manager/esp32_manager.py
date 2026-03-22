import rclpy
from rclpy.node import Node
import subprocess
import serial
import time

class ESP32Manager(Node):
    def __init__(self):
        super().__init__('esp32_manager')
        
        # 1. รีเซ็ตบอร์ด ESP32 ผ่าน DTR/RTS
        self.reset_esp32('/dev/ttyUSB0')
        self.reset_esp32('/dev/ttyUSB1')
        
        # 2. รัน micro-ROS Agent (ใช้ subprocess แบบ non-blocking)
        self.get_logger().info("Starting micro-ROS Agents...")
        
        self.agent0 = self.start_agent('/dev/ttyUSB0', '115200')
        self.agent1 = self.start_agent('/dev/ttyUSB1', '115200')

    def reset_esp32(self, port):
        try:
            self.get_logger().info(f"Resetting ESP32 on {port}...")
            ser = serial.Serial(port)
            ser.setDTR(False)  # IO0 high
            ser.setRTS(True)   # EN low (Reset)
            time.sleep(0.1)
            ser.setRTS(False)  # EN high (Boot)
            ser.close()
        except Exception as e:
            self.get_logger().error(f"Failed to reset {port}: {e}")

    def start_agent(self, port, baud):
        cmd = [
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', port, '-b', baud
        ]
        return subprocess.Popen(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Manager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.agent0.terminate()
        node.agent1.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()