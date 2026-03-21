#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray
import joblib
import os
import numpy as np
import warnings

warnings.filterwarnings("ignore", category=UserWarning)

class Check_floor(Node):
    def __init__(self):
        super().__init__('Check_floor')
        
        # 1. โหลด Model และ Scaler
        package_name = 'my_manager' 
        try:
            data_dir = os.path.join(get_package_share_directory(package_name), 'models')
            model_path = os.path.join(data_dir, 'floor_svm_model.pkl')
            scaler_path = os.path.join(data_dir, 'floor_scaler.pkl')
            
            self.model = joblib.load(model_path)
            self.scaler = joblib.load(scaler_path)
            self.get_logger().info("✅ โหลดสมอง AI (SVM) สำเร็จ!")
        except Exception as e:
            self.get_logger().error(f"❌ ไม่สามารถโหลดโมเดลได้: {e}")
            raise SystemExit
            
        # 2. Subscribe ข้อมูลเซนเซอร์ (Real-time)
        self.sub_sensors = self.create_subscription(
            Float32MultiArray,
            'sensors',
            self.sensors_array_callback,
            10)
        
        # 3. Subscribe ข้อมูลเป้าหมายจาก Whisper
        self.sub_target = self.create_subscription(
            Float32MultiArray,
            'robot_target',
            self.target_callback,
            10)

        # 4. Publisher สำหรับส่งผลการตรวจสอบ
        self.pub_check_floor = self.create_publisher(
            Float32MultiArray,
            'check_floor',
            10)

        self.current_floor = None
        self.get_logger().info("🚀 AI Floor Predictor พร้อมทำงาน...")

    def predict_floor(self, p, t):
        try:
            X_live = np.array([[p, t]], dtype=np.float32)
            X_live_scaled = self.scaler.transform(X_live)
            prediction = self.model.predict(X_live_scaled)
            return int(prediction[0])
        except Exception as e:
            return None

    def target_callback(self, msg):
        """ เมื่อได้รับคำสั่งเป้าหมาย ให้ทำการเปรียบเทียบและส่ง Topic """
        if len(msg.data) >= 3 and self.current_floor is not None:
            target_val = float(msg.data[2])
            current_f = float(self.current_floor)
            
            self.get_logger().info(f"🎯 ตำแหน่ง target : {target_val}")
            
            check_msg = Float32MultiArray()
            
            if target_val == current_f:
                # กรณีชั้นเดียวกัน
                self.get_logger().info("✅ ชั้นเดียวกัน")
                check_msg.data = [0.0]
            
            elif target_val > current_f:
                # กรณีเป้าหมายอยู่สูงกว่า
                self.get_logger().info(f"🔼 เป้าหมาย {target_val} > ตำแหน่งจริง {current_f} : ขึ้นบันได")
                check_msg.data = [current_f, 1.0]
                
            elif target_val < current_f:
                # กรณีเป้าหมายอยู่ต่ำกว่า
                self.get_logger().info(f"🔽 เป้าหมาย {target_val} < ตำแหน่งจริง {current_f} : ลงบันได")
                check_msg.data = [current_f, -1.0]
            
            self.pub_check_floor.publish(check_msg)
        else:
            self.get_logger().warn("⚠️ ยังไม่มีข้อมูลชั้นจริงหรือเป้าหมายไม่ครบ")

    def sensors_array_callback(self, msg):
        """ อัปเดตและโชว์ตำแหน่งจริงตลอดเวลา """
        if len(msg.data) >= 11:
            p = msg.data[9]
            t = msg.data[10]
            
            predicted = self.predict_floor(p, t)
            if predicted is not None:
                self.current_floor = predicted
                # โชว์ตำแหน่งจริงตลอดเวลา (ใช้ throttle เพื่อไม่ให้ log วิ่งไวเกินไป)
                self.get_logger().info(f"📍 ตำแหน่งชั้นจริง : {self.current_floor}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = Check_floor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 ปิดระบบ AI Predictor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()