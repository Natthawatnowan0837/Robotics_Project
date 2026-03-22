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
        """ เมื่อได้รับคำสั่งเป้าหมาย ให้รวมข้อมูล X, Y และสถานะชั้นส่งไปที่ check_floor """
        # ตรวจสอบว่ามีข้อมูล X, Y, Target_Floor และ AI คำนวณชั้นปัจจุบันได้แล้ว
        if len(msg.data) >= 3 and self.current_floor is not None:
            target_x = float(msg.data[0])
            target_y = float(msg.data[1])
            target_f = float(msg.data[2])
            current_f = float(self.current_floor)
            
            check_msg = Float32MultiArray()
            
            # ตรรกะการเช็คชั้นและการรวมข้อมูล [X, Y, Current_Floor, Status]
            if target_f == current_f:
                # กรณีชั้นเดียวกัน
                self.get_logger().info(f"✅ อยู่ชั้น {current_f} เหมือนกัน (ไปที่ X:{target_x}, Y:{target_y})")
                check_msg.data = [target_x, target_y, current_f, 0.0]
            
            elif target_f > current_f:
                # กรณีเป้าหมายอยู่สูงกว่า -> ส่งสถานะ 1.0 (ขึ้น)
                self.get_logger().info(f"🔼 เป้าหมายชั้น {target_f} > จริง {current_f} : ไปบันไดเพื่อขึ้น")
                check_msg.data = [target_x, target_y, current_f, 1.0]
                
            elif target_f < current_f:
                # กรณีเป้าหมายอยู่ต่ำกว่า -> ส่งสถานะ -1.0 (ลง)
                self.get_logger().info(f"🔽 เป้าหมายชั้น {target_f} < จริง {current_f} : ไปบันไดเพื่อลง")
                check_msg.data = [target_x, target_y, current_f, -1.0]
            
            # ส่งข้อมูลรวมทั้งหมดออกไป
            self.pub_check_floor.publish(check_msg)
        else:
            if self.current_floor is None:
                self.get_logger().warn("⚠️ AI ยังระบุชั้นปัจจุบันไม่ได้ (รอข้อมูลจาก sensors)", throttle_duration_sec=2.0)
            else:
                self.get_logger().warn("⚠️ ข้อมูล robot_target ไม่ครบ (ต้องการ X, Y, Floor)", throttle_duration_sec=2.0)
                
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