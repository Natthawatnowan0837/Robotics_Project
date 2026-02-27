import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class PIDVisualizer(Node):
    def __init__(self):
        super().__init__('pid_visualizer')
        self.sub_out = self.create_subscription(Float32MultiArray, 'wheel_velocity_output', self.out_cb, 10)
        self.sub_set = self.create_subscription(Float32MultiArray, 'wheel_setpoint', self.set_cb, 10)
        
        self.out_val = [0.0, 0.0]
        self.set_val = [0.0, 0.0]
        self.history_out = []
        self.history_set = []

    def out_cb(self, msg): self.out_val = msg.data
    def set_cb(self, msg): 
        self.set_val = msg.data
        # เก็บข้อมูลเพื่อพล็อต (แสดงตัวอย่างเฉพาะล้อซ้าย)
        self.history_out.append(self.out_val[0])
        self.history_set.append(self.set_val[0])
        if len(self.history_out) > 100: # เก็บแค่ 100 จุดล่าสุด
            self.history_out.pop(0)
            self.history_set.pop(0)
        self.update_plot()

    def update_plot(self):
        plt.clf()
        plt.plot(self.history_out, label='Actual Velocity (L)')
        plt.plot(self.history_set, label='Setpoint (L)', linestyle='--')
        plt.legend()
        plt.pause(0.01)

def main():
    plt.ion() # เปิดโหมด Interactive
    rclpy.init()
    node = PIDVisualizer()
    rclpy.spin(node)
    plt.show()