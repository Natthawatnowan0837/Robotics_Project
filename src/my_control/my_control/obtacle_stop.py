import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

class CameraYoloNode(Node):
    def __init__(self):
        super().__init__('camera_yolo_node')
        self.bridge = CvBridge()
        self.depth_image = None
        
        # --- [ 1. โหลดโมเดล ] ---
        self.model_path = '/home/noone/Robotics_Project/best.pt'
        self.hub_path = '/home/noone/.cache/torch/hub/ultralytics_yolov5_master'
        
        try:
            self.model = torch.hub.load(self.hub_path, 'custom', 
                                        path=self.model_path, 
                                        source='local')
            self.model.conf = 0.5
            self.get_logger().info('YOLOv5 Model Loaded!')
        except Exception as e:
            self.get_logger().error(f'Model load failed: {e}')
            self.model = None

        # --- [ 2. Subscriptions ] ---
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        
        self.frame_count = 0
        self.last_results = None 
        self.get_logger().info('Node Ready: Safety Logic with Lane Lines')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

    def color_callback(self, msg):
        if self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_image.shape
            self.frame_count += 1
            stop_warning = False

            # --- [ 3. วาดเส้นแบ่งเลนแนวตั้ง (พื้นที่ตรงกลาง 400px) ] ---
            mid_x = w // 2
            left_line = mid_x - 250
            right_line = mid_x + 250
            
            # วาดเส้นประหรือเส้นทึบสีขาว (BGR: 255, 255, 255)
            cv2.line(cv_image, (left_line, 0), (left_line, h), (255, 255, 255), 2)
            cv2.line(cv_image, (right_line, 0), (right_line, h), (255, 255, 255), 2)

            # 4. Inference ทุก 3 เฟรม
            if self.frame_count % 3 == 0:
                input_img = cv2.resize(cv_image, (640, 480))
                results = self.model(input_img)
                self.last_results = results.pandas().xyxy[0]

            # 5. ประมวลผลผลลัพธ์
            if self.last_results is not None:
                ratio_x = w / 640
                ratio_y = h / 480

                for _, row in self.last_results.iterrows():
                    xmin, ymin = int(row['xmin'] * ratio_x), int(row['ymin'] * ratio_y)
                    xmax, ymax = int(row['xmax'] * ratio_x), int(row['ymax'] * ratio_y)
                    center_x, center_y = (xmin + xmax) // 2, (ymin + ymax) // 2
                    
                    distance_m = -1.0
                    if self.depth_image is not None:
                        if 0 <= center_y < self.depth_image.shape[0] and 0 <= center_x < self.depth_image.shape[1]:
                            dist_val = self.depth_image[center_y, center_x]
                            if dist_val > 0:
                                distance_m = float(dist_val) / 1000.0
                    
                    # Safety Logic (1.8 เมตร)
                    color = (0, 255, 0)
                    if 0 < distance_m < 1.8:
                        color = (0, 0, 255)
                        stop_warning = True
                    
                    label = f"{row['name']} {distance_m:.2f}m" if distance_m > 0 else f"{row['name']} Unknown"
                    cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), color, 2)
                    cv2.putText(cv_image, label, (xmin, ymin - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 6. แสดงคำเตือน STOP
            if stop_warning:
                cv2.rectangle(cv_image, (0, 0), (w, 60), (0, 0, 255), -1)
                cv2.putText(cv_image, "!!! STOP - OBJECT TOO CLOSE !!!", (50, 45), 
                            cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 2)

            cv2.imshow("Safety Monitor", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraYoloNode()
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