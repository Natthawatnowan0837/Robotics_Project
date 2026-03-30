#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # ต้องติดตั้ง python3-cv-bridge
import numpy as np
import cv2
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class StairAlignerDebug(Node):
    def __init__(self):
        super().__init__('stair_aligner_debug')
        
        self.bridge = CvBridge()
        
        # --- [ Parameters ] ---
        self.target_dist      = 0.45   
        self.angle_tolerance  = 10.0  
        self.conf_threshold   = 60.0  
        self.roi_height_ratio = 0.5   # 0.5 คือครึ่งล่างของภาพ
        self.intrinsics = None

        # --- [ Publishers & Subscribers ] ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_rotation', 10)
        
        # เปลี่ยนเป็น Image (Raw) แทน CompressedImage
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.image_callback, 
            10
        )
        
        # ตัวอย่างนี้ใช้ Image Raw ในการแสดงผล แต่การคำนวณระยะต้องใช้ Depth
        # หากคุณต้องการคำนวณจาก Depth ด้วย ให้ Sub Topic Depth Raw ควบคู่ไปด้วย
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        )

        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/camera/camera/depth/camera_info', 
            self.info_callback, 
            10
        )

        self.current_color_frame = None
        self.get_logger().info("🔍 Debug Mode: Using RAW Image with ROI Visualization")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            self.intrinsics = True

    def image_callback(self, msg):
        # เก็บภาพ Color ไว้เพื่อใช้ในการแสดงผล (Visualization) เท่านั้น
        self.current_color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        if self.intrinsics is None or self.current_color_frame is None: 
            return
        
        try:
            # 1. แปลง Depth Message (Raw) เป็น OpenCV (Uint16)
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            h, w = cv_depth.shape
            start_row = int(h * self.roi_height_ratio)

            # 2. เตรียมภาพสำหรับ Debug โดยใช้ภาพสีจาก image_raw
            debug_img = self.current_color_frame.copy()

            # 3. วาดกรอบ ROI และพื้นที่ที่นำมาคำนวณ (เปอร์เซ็นต์พื้นที่)
            # วาดเส้นแบ่งพื้นที่คำนวณ
            cv2.line(debug_img, (0, start_row), (w, start_row), (0, 255, 0), 2)
            # ระบายสีโปร่งแสงในพื้นที่ ROI เพื่อให้เห็นชัดว่าจุดไหนถูกคำนวณ
            overlay = debug_img.copy()
            cv2.rectangle(overlay, (0, start_row), (w, h), (0, 255, 0), -1)
            cv2.addWeighted(overlay, 0.3, debug_img, 0.7, 0, debug_img)

            # 4. การคำนวณฟีเจอร์บันได
            cropped_depth = cv_depth[start_row:, :]
            stride = 20 # เพิ่ม Stride เพื่อไม่ให้กระตุก
            reduced_depth = cropped_depth[::stride, ::stride]
            z = reduced_depth.astype(np.float32) / 1000.0
            mask = (z > 0.3) & (z < 3.0)

            status_text = "Searching..."
            if np.any(mask):
                v, u = np.indices(reduced_depth.shape)
                u_real, v_real = u[mask] * stride, (v[mask] * stride) + start_row
                z_valid = z[mask]
                
                # แปลง 2D Image -> 3D Camera Space
                x_cam = (u_real - self.cx) * z_valid / self.fx
                y_cam = (v_real - self.cy) * z_valid / self.fy
                points = np.stack((z_valid, -x_cam, -y_cam), axis=-1)

                data = self.extract_stair_features(points)
                if data:
                    _, yaw_deg, confidence, dist = data
                    status_text = f"CONF: {confidence:.1f}% | DIST: {dist:.2f}m | YAW: {yaw_deg:.1f}"
                    
                    # วาดจุดกึ่งกลางหรือทิศทางบนจอ
                    cv2.circle(debug_img, (w//2, h//2 + 50), 10, (255, 255, 255), -1)

            # 5. แสดงผล Text และ GUI
            cv2.putText(debug_img, f"ROI: {int((1-self.roi_height_ratio)*100)}% of frame", 
                        (10, start_row - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_img, status_text, (15, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow("Stair Alignment Debugger", debug_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Logic Error: {e}")

    def extract_stair_features(self, points):
        # RANSAC เพื่อหาระนาบบันได
        best_inliers_count = 0
        best_normal = None
        
        # สุ่ม 20 ครั้งเพื่อหาพื้นผิวที่ใหญ่ที่สุดใน ROI
        for _ in range(20):
            if len(points) < 3: break
            idx = np.random.choice(len(points), 3, replace=False)
            p1, p2, p3 = points[idx]
            normal = np.cross(p2-p1, p3-p1)
            norm = np.linalg.norm(normal)
            if norm < 1e-4: continue
            normal /= norm
            
            # กรองให้เอาเฉพาะระนาบที่หันหน้าเข้าหาหุ่นยนต์ (Normal Z ต้องเป็นลบใน Camera Link)
            if normal[0] > 0: normal *= -1
            if abs(normal[0]) < 0.8: continue # ต้องเป็นแผ่นที่ค่อนข้างตั้งฉากกับพื้น

            d = -np.dot(normal, p1)
            inliers = np.sum(np.abs(np.dot(points, normal) + d) < 0.04)
            if inliers > best_inliers_count:
                best_inliers_count = inliers
                best_normal = normal
        
        if best_inliers_count > 15:
            yaw_deg = math.degrees(math.atan2(best_normal[1], -best_normal[0]))
            dist = np.mean(points[:, 0])
            # เปอร์เซ็นต์ความเชื่อมั่น (Inliers เทียบกับจุดทั้งหมดใน ROI)
            confidence = min((best_inliers_count / (len(points)*0.5)) * 100, 100.0)
            return True, yaw_deg, confidence, dist
        return None

def main(args=None):
    rclpy.init(args=args)
    node = StairAlignerDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()