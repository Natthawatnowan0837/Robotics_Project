import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray # สมมติว่า msg_sensors เป็น type นี้
import numpy as np
import math

class Mpu6050EKFNode(Node):
    def __init__(self):
        super().__init__('mpu6050_ekf_node')
        
        # --- [ 1. ตั้งค่า Subscriber ] ---
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensors',  # ชื่อ Topic
            self.listener_callback,
            10)
        
        # --- [ 2. ตัวแปร EKF ] ---
        # State Vector x = [roll, roll_bias, pitch, pitch_bias]^T
        self.x = np.zeros((4, 1))
        
        # Error Covariance Matrix P (ความเชื่อมั่นเริ่มต้น)
        self.P = np.eye(4) * 0.1
        
        # Process Noise Q (ความเพี้ยนของ Model/Gyro)
        # ปรับค่าน้อยลงถ้าต้องการให้เข็มทิศนิ่งขึ้น
        q_angle = 0.001
        q_bias = 0.003
        self.Q = np.diag([q_angle, q_bias, q_angle, q_bias])
        
        # Measurement Noise R (ความเพี้ยนของ Accelerometer)
        # ปรับค่ามากขึ้นถ้า Sensor สั่นแรง (Vibration)
        r_val = 0.05
        self.R = np.diag([r_val, r_val])
        
        # Time Management
        self.last_time = self.get_clock().now()
        self.is_initialized = False

    def listener_callback(self, msg):
        # ข้อมูลจาก Arduino ตาม Index ที่ระบุมา:
        # data[0-2]: accel_x, y, z
        # data[3-5]: gyro_x, y, z (rad/s)
        # data[6-7]: platform_x, y (ไม่ได้ใช้ใน EKF นี้)
        
        ax, ay, az = msg.data[0], msg.data[1], msg.data[2]
        gx, gy, gz = msg.data[3], msg.data[4], msg.data[5]

        # คำนวณ dt (Delta Time)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if not self.is_initialized:
            # กำหนดค่าเริ่มต้นจาก Accelerometer
            roll_init = math.atan2(ay, az)
            pitch_init = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            self.x[0, 0] = roll_init
            self.x[2, 0] = pitch_init
            self.is_initialized = True
            return

        # --- [ STEP 1: PREDICT ] ---
        # เราใช้ Gyroscope ในการทำนายมุมถัดไป
        # roll_new = roll + dt * (gx - roll_bias)
        # pitch_new = pitch + dt * (gy - pitch_bias)
        
        # State Transition Matrix (F)
        F = np.array([
            [1, -dt, 0,  0],
            [0,  1,  0,  0],
            [0,  0,  1, -dt],
            [0,  0,  0,  1]
        ])
        
        # Control Input Matrix (B) - ใช้ค่าความเร็วเชิงมุมจาก Gyro
        B = np.array([
            [dt, 0],
            [0,  0],
            [0, dt],
            [0,  0]
        ])
        u = np.array([[gx], [gy]])
        
        # Prediction Update
        self.x = F @ self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q

        # --- [ STEP 2: UPDATE ] ---
        # ใช้ Accelerometer มาแก้ค่าที่ Predict ไว้
        
        # คำนวณมุมจริงจาก Accel
        z_roll = math.atan2(ay, az)
        z_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        z = np.array([[z_roll], [z_pitch]])
        
        # Measurement Matrix (H) - เราวัดเฉพาะ Roll และ Pitch (Index 0 และ 2)
        H = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])
        
        # Innovation (ความต่าง)
        y = z - (H @ self.x)
        
        # Kalman Gain (K)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Final Update
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

        # --- [ Output ] ---
        final_roll_deg = math.degrees(self.x[0, 0])
        final_pitch_deg = math.degrees(self.x[2, 0])
        
        self.get_logger().info(f'EKF -> Roll: {final_roll_deg:.2f}, Pitch: {final_pitch_deg:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()