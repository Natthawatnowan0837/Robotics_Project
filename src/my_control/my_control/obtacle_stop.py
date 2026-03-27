#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

os.environ['ULTRALYTICS_OFFLINE'] = 'True' 
from ultralytics import YOLO 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class YoloV8UltraFastNode(Node):
    def __init__(self):
        super().__init__('yolo_v8_ultra_fast_node')
        self.bridge = CvBridge()
        
        # ==========================================
        # [ ส่วนที่แก้ไขง่าย - CONFIGURATION ]
        # ==========================================
        self.model_path = '/home/noone/Robotics_Project/best.pt'
        self.roi_w_ratio = 0.7       # ความกว้าง 70%
        self.roi_h_ratio = 1.0       # ความสูง 100%
        self.safety_threshold = 1.5  # ระยะเบรก (เมตร)
        self.inference_interval = 4  
        self.ai_imgsz = 160          
        # ==========================================

        self.model = YOLO(self.model_path)
        self.frame_count = 0
        self.last_results = None
        self.depth_image = None

        qos_fast = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.depth_sub = self.create_subscription(CompressedImage, '/camera/camera/depth/image_rect_raw/compressedDepth', self.depth_callback, qos_fast)
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        
        self.get_logger().info('✅ Distance will now show at Box Center')

    def depth_callback(self, msg):
        try:
            depth_data = np.frombuffer(msg.data, np.uint8)
            self.depth_image = cv2.imdecode(depth_data[12:], cv2.IMREAD_UNCHANGED)
        except: pass

    def color_callback(self, msg):
        if self.model is None or self.depth_image is None: return

        try:
            full_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = full_img.shape
            stop_warning = False

            # --- [ STEP 1: ROI ] ---
            x1_roi = int(w * (1 - self.roi_w_ratio) / 2)
            y1_roi = int(h * (1 - self.roi_h_ratio) / 2)
            x2_roi = x1_roi + int(w * self.roi_w_ratio)
            y2_roi = y1_roi + int(h * self.roi_h_ratio)
            
            cv2.rectangle(full_img, (x1_roi, y1_roi), (x2_roi, y2_roi), (200, 200, 200), 1)

            roi_img = full_img[y1_roi:y2_roi, x1_roi:x2_roi]
            
            self.frame_count += 1
            if self.frame_count % self.inference_interval == 0:
                self.last_results = self.model.predict(roi_img, imgsz=self.ai_imgsz, conf=0.5, verbose=False)

            # --- [ STEP 2: Process Result & Distance ] ---
            if self.last_results is not None:
                for result in self.last_results:
                    for box in result.boxes:
                        bx1, by1, bx2, by2 = map(int, box.xyxy[0])
                        gx1, gy1, gx2, gy2 = bx1 + x1_roi, by1 + y1_roi, bx2 + x1_roi, by2 + y1_roi
                        
                        # คำนวณจุดกึ่งกลาง (Center of Box)
                        cx, cy = (gx1 + gx2) // 2, (gy1 + gy2) // 2
                        
                        # ดึงค่า Depth (Median ในพื้นที่เล็กๆ รอบจุดศูนย์กลาง 5x5 เพื่อความแม่นยำ)
                        obj_depth = self.depth_image[max(0,cy-2):min(h,cy+2), max(0,cx-2):min(w,cx+2)]
                        valid_depths = obj_depth[obj_depth > 0]
                        
                        if len(valid_depths) > 0:
                            distance_m = np.median(valid_depths) / 1000.0
                            
                            color = (0, 255, 0)
                            if distance_m < self.safety_threshold:
                                color = (0, 0, 255)
                                stop_warning = True
                            
                            # วาดกรอบ Boxing
                            cv2.rectangle(full_img, (gx1, gy1), (gx2, gy2), color, 2)
                            
                            # วาดจุดกึ่งกลาง (Center Dot)
                            cv2.circle(full_img, (cx, cy), 3, (255, 255, 255), -1)
                            
                            # --- [ หัวใจสำคัญ: แสดง Distance กลาง Box ] ---
                            dist_label = f"{distance_m:.2f}m"
                            # คำนวณขนาดตัวอักษรเพื่อให้จัดวางตรงกลางได้สวย
                            (text_w, text_h), _ = cv2.getTextSize(dist_label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                            cv2.putText(full_img, dist_label, (cx - text_w // 2, cy + text_h // 2), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            
                            # แสดงชื่อวัตถุไว้ที่หัว Box เหมือนเดิม (ตัวเล็กหน่อย)
                            name_label = f"{self.model.names[int(box.cls[0])]}"
                            cv2.putText(full_img, name_label, (gx1, gy1 - 5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # --- [ STEP 3: Warning UI ] ---
            if stop_warning:
                cv2.rectangle(full_img, (0, 0), (w, 50), (0, 0, 255), -1)
                cv2.putText(full_img, "!!! OBSTACLE !!!", (w//2 - 100, 35), 
                            cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 2)

            display_img = cv2.resize(full_img, (0,0), fx=0.8, fy=0.8)
            cv2.imshow("Optimized AI Monitor", display_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloV8UltraFastNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()