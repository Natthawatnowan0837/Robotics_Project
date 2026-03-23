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

class FloorManagerNode(Node):
    def __init__(self):
        super().__init__('floor_manager_node')
        
        # 1. Load Model และ Scaler (SVM)
        package_name = 'my_manager' 
        try:
            # ตรวจสอบตำแหน่งไฟล์ Model ให้ถูกต้อง
            pkg_share = get_package_share_directory(package_name)
            model_path = os.path.join(pkg_share, 'models', 'floor_svm_model.pkl')
            scaler_path = os.path.join(pkg_share, 'models', 'floor_scaler.pkl')
            
            self.model = joblib.load(model_path)
            self.scaler = joblib.load(scaler_path)
            self.get_logger().info("✅ SVM Model & Scaler Loaded Successfully!")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load models: {e}")
            raise SystemExit
            
        # 2. Subscribers
        # รับค่าเซนเซอร์ [..., pressure(9), temp(10)] เพื่อทำนายชั้นปัจจุบัน
        self.sub_sensors = self.create_subscription(
            Float32MultiArray, 'sensors', self.sensors_callback, 10)
        
        # เปลี่ยนมารับค่า "ชั้นเป้าหมาย" จาก /check_floor
        self.sub_check_floor = self.create_subscription(
            Float32MultiArray, '/check_floor', self.check_floor_callback, 10)

        # 3. Publisher
        # ส่งผลสรุป [current_floor, status]
        self.pub_update_floor = self.create_publisher(
            Float32MultiArray, '/update_floor', 10)

        self.current_floor = None
        self.get_logger().info("🚀 Floor Manager Ready: Waiting for /check_floor and sensors...")

    def predict_floor(self, p, t):
        """ ใช้ SVM ทำนายเลขชั้นจากค่าความกดอากาศและอุณหภูมิ """
        try:
            X_live = np.array([[p, t]], dtype=np.float32)
            X_live_scaled = self.scaler.transform(X_live)
            prediction = self.model.predict(X_live_scaled)
            return int(prediction[0])
        except Exception as e:
            self.get_logger().error(f"Prediction Error: {e}")
            return None

    def sensors_callback(self, msg):
        """ ทำนายชั้นปัจจุบันจากเซนเซอร์ index 9 และ 10 """
        if len(msg.data) >= 11:
            p, t = msg.data[9], msg.data[10]
            predicted = self.predict_floor(p, t)
            if predicted is not None:
                self.current_floor = predicted
                # Log ชั้นปัจจุบันที่ AI เห็น
                self.get_logger().info(f"📍 Current Floor (AI): {self.current_floor}", throttle_duration_sec=5.0)

    def check_floor_callback(self, msg):
        """ 
        รับชั้นเป้าหมายจาก /check_floor แล้วเทียบกับชั้นที่ AI ทำนายได้
        msg.data[0] คือ target_floor
        """
        if self.current_floor is None:
            self.get_logger().warn("⏳ AI still identifying current floor. Please wait...", throttle_duration_sec=3.0)
            return

        if len(msg.data) >= 1:
            target_f = float(msg.data[0])
            current_f = float(self.current_floor)
            status = 0.0 
            
            # ตรวจสอบสถานะ
            if target_f == current_f:
                status = 0.0
                self.get_logger().info(f"✅ Correct Floor: {current_f}")
            elif target_f > current_f:
                status = 1.0
                self.get_logger().info(f"🔼 Target {target_f} > AI Predicted {current_f}: NEED TO GO UP")
            elif target_f < current_f:
                status = -1.0
                self.get_logger().info(f"🔽 Target {target_f} < AI Predicted {current_f}: NEED TO GO DOWN")
            
            # Publish ผลลัพธ์ [ชั้นที่ AI เห็น, สถานะ -1/0/1]
            update_msg = Float32MultiArray()
            update_msg.data = [current_f, status]
            self.pub_update_floor.publish(update_msg)
        else:
            self.get_logger().warn("⚠️ Received empty data from /check_floor")

def main(args=None):
    rclpy.init(args=args)
    node = FloorManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopping Floor Manager")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()