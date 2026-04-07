#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray
from my_command.srv import CheckFloor  
import joblib
import os
import numpy as np
import warnings

warnings.filterwarnings("ignore", category=UserWarning)

class Check_floor(Node):
    def __init__(self):
        super().__init__('floor_manager_node')
        
        # 1. Load Model และ Scaler (SVM)
        package_name = 'my_manager' 
        try:
            pkg_share = get_package_share_directory(package_name)
            model_path = os.path.join(pkg_share, 'models', 'floor_svm_model.pkl')
            scaler_path = os.path.join(pkg_share, 'models', 'floor_scaler.pkl')
            
            self.model = joblib.load(model_path)
            self.scaler = joblib.load(scaler_path)
            self.get_logger().info("✅ SVM Model & Scaler Loaded Successfully!")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load models: {e}")
            raise SystemExit
            
        # 2. Subscriber สำหรับอ่านค่า Sensor
        self.sub_sensors = self.create_subscription(
            Float32MultiArray, 'sensors', self.sensors_callback, 10)

        # 3. Service Server
        self.srv = self.create_service(
            CheckFloor, 'check_floor_service', self.check_floor_callback)

        self.current_floor = None
        self.get_logger().info("🚀 Floor Service Server Ready: Waiting for requests...")

    def predict_floor(self, p, t):
        try:
            X_live = np.array([[p, t]], dtype=np.float32)
            X_live_scaled = self.scaler.transform(X_live)
            prediction = self.model.predict(X_live_scaled)
            return float(prediction[0])
        except Exception as e:
            self.get_logger().error(f"Prediction Error: {e}")
            return None

    def sensors_callback(self, msg):
        """ ทำนายชั้นปัจจุบันและโชว์ค่าใน Logger ตลอดเวลา """
        if len(msg.data) >= 11:
            # ดึงค่า pressure (index 9) และ temp (index 10)
            p, t = msg.data[9], msg.data[10]
            predicted = self.predict_floor(p, t)
            
            if predicted is not None:
                self.current_floor = predicted
                # --- ส่วนที่เพิ่ม: โชว์ค่าที่ทำนายได้ตลอดเวลา (ทุกๆ 2 วินาที) ---
                self.get_logger().info(
                    f"📊 AI Live Prediction: Floor {self.current_floor} (P:{p:.2f}, T:{t:.2f})", 
                    throttle_duration_sec=2.0
                )

    def check_floor_callback(self, request, response):
        if self.current_floor is None:
            response.current_floor = -1.0
            response.status = "error_identifying"
            return response

        target_f = request.floor
        current_f = self.current_floor
        response.current_floor = current_f

        if target_f == current_f:
            response.status = "same_floor"
            self.get_logger().info(f"🎯 Service Result: same_floor ({current_f})")
        elif target_f > current_f:
            response.status = "Up"
            self.get_logger().info(f"🎯 Service Result: up (Target:{target_f} > Current:{current_f})")
        elif target_f < current_f:
            response.status = "Down"
            self.get_logger().info(f"🎯 Service Result: down (Target:{target_f} < Current:{current_f})")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Check_floor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopping Floor Manager Server")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()