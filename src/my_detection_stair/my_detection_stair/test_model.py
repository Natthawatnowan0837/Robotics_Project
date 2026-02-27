#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# เพิ่ม message type สำหรับภาพสี
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import numpy as np
import open3d as o3d
import torch
import torch.nn as nn
import torch.nn.functional as F
import os
import cv2  # เพิ่ม OpenCV สำหรับการแสดงผลภาพ

# ==========================================
# 1. นิยามโครงสร้างโมเดล (TNet & PointNet)
# ==========================================
# [ Copy คลาส TNet และ PointNet ทั้งหมดของคุณมาวางตรงนี้ ]
class TNet(nn.Module):
    # ... โค้ดเดิมของคุณ ...
    def __init__(self, k=3):
        super(TNet, self).__init__()
        self.k = k
        self.conv1 = nn.Conv1d(k, 64, 1)
        self.conv2 = nn.Conv1d(64, 128, 1)
        self.conv3 = nn.Conv1d(128, 1024, 1)
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, k*k)
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)

    def forward(self, x):
        batch_size = x.size(0)
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)
        x = F.relu(self.bn4(self.fc1(x)))
        x = F.relu(self.bn5(self.fc2(x)))
        x = self.fc3(x)
        iden = torch.eye(self.k).repeat(batch_size, 1, 1)
        if x.is_cuda: iden = iden.cuda()
        x = x.view(-1, self.k, self.k) + iden
        return x

class PointNet(nn.Module):
    # ... โค้ดเดิมของคุณ ...
    def __init__(self, classes=3):
        super(PointNet, self).__init__()
        self.stn = TNet(k=3)
        self.conv1 = nn.Conv1d(3, 64, 1)
        self.bn1 = nn.BatchNorm1d(64)
        self.fstn = TNet(k=64)
        self.conv2 = nn.Conv1d(64, 64, 1)
        self.bn2 = nn.BatchNorm1d(64)
        self.conv3 = nn.Conv1d(64, 64, 1)
        self.bn3 = nn.BatchNorm1d(64)
        self.conv4 = nn.Conv1d(64, 1024, 1)
        self.bn4 = nn.BatchNorm1d(1024)
        self.fc1 = nn.Linear(1024, 512)
        self.bn5 = nn.BatchNorm1d(512)
        self.fc2 = nn.Linear(512, 256)
        self.bn6 = nn.BatchNorm1d(256)
        self.fc3 = nn.Linear(256, classes)
        self.dropout = nn.Dropout(p=0.3)

    def forward(self, x):
        trans = self.stn(x)
        x = x.transpose(2, 1)
        x = torch.bmm(x, trans)
        x = x.transpose(2, 1)
        x = F.relu(self.bn1(self.conv1(x)))
        trans_feat = self.fstn(x)
        x = x.transpose(2, 1)
        x = torch.bmm(x, trans_feat)
        x = x.transpose(2, 1)
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = self.bn4(self.conv4(x))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)
        x = F.relu(self.bn5(self.fc1(x)))
        x = F.relu(self.bn6(self.fc2(x)))
        x = self.dropout(x)
        x = self.fc3(x)
        return F.log_softmax(x, dim=1), trans_feat

# ==========================================
# 2. ROS 2 Node สำหรับ Inference และแสดงผล CV
# ==========================================
class StairInferenceNode(Node):
    def __init__(self):
        super().__init__('stair_inference_node')
        
        self.bridge = CvBridge()
        self.intrinsics = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # สำหรับเก็บภาพสีล่าสุดเพื่อรอนำมาวาด Label
        self.latest_color_img = None
        
        # --- โหลดโมเดลด้วย Dynamic Path ---
        package_name = 'my_detection_stair'
        try:
            share_dir = get_package_share_directory(package_name)
            self.model_path = os.path.join(share_dir, 'pointnet_stairs.pth')
        except:
            self.model_path = 'pointnet_stairs.pth'

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model not found! Checked: {self.model_path}")
            return

        self.model = PointNet(classes=3).to(self.device)
        self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
        self.model.eval()
        
        self.get_logger().info(f"Model Ready: {self.model_path} on {self.device}")
        
        # Labels mapping และ การตั้งค่าสีสำหรับวาด Box (BGR format)
        self.labels_map = {0: "DOWNSTAIRS", 1: "UPSTAIRS", 2: "OTHERS"}
        self.label_colors = {
            0: (0, 0, 255),    # สีแดงสำหรับ Downstairs
            1: (0, 255, 255),  # สีเหลืองสำหรับ Upstairs
            2: (255, 255, 255) # สีขาวสำหรับ Others
        }

        # กำหนดเกณฑ์ Confidence ขั้นต่ำ (เช่น 80%)
        self.confidence_threshold = 80.0

        # ROS 2 Subscribers
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.info_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        
        # เพิ่ม Subscriber สำหรับภาพสี (RGB/BGR) เพื่อนำมาวาด Box แสดงผล
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        
        self.pcd_pub = self.create_publisher(PointCloud2, '/inference_point_cloud', 10)

        # สร้างหน้าต่าง OpenCV รอล่วงหน้า
        cv2.namedWindow("Stair Detection result", cv2.WINDOW_NORMAL)

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic()
            self.intrinsics.set_intrinsics(msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def color_callback(self, msg):
        # เก็บภาพสีล่าสุดไว้แปลงและวาดใน depth_callback
        try:
            self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")

    def normalize_point_cloud(self, points):
        centroid = np.mean(points, axis=0)
        points_centered = points - centroid
        max_distance = np.max(np.linalg.norm(points_centered, axis=1))
        if max_distance == 0: max_distance = 1.0
        return points_centered / max_distance

    def draw_label_box(self, img, label_text, color):
        """วาดกล่องข้อความจำลองที่มุมภาพ"""
        h, w, _ = img.shape
        # กำหนดขนาดและตำแหน่งของกล่อง (เช่น มุมบนซ้าย)
        box_coords = ((10, 10), (w // 2, 60)) # (x1, y1), (x2, y2)
        text_pos = (20, 45)
        
        # วาดกล่องพื้นหลัง (Filled rectangle)
        cv2.rectangle(img, box_coords[0], box_coords[1], color, cv2.FILLED)
        
        # วาดข้อความ "คำตอบ:" และผลลัพธ์
        full_text = f"RESULT: {label_text}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        font_thickness = 2
        text_color = (0, 0, 0) # สีดำเพื่อให้ตัดกับสีพื้นหลัง
        
        cv2.putText(img, full_text, text_pos, font, font_scale, text_color, font_thickness, cv2.LINE_AA)

    def depth_callback(self, msg):
        if self.intrinsics is None: return

        try:
            # --- 1. Point Cloud Preprocessing (ส่วนเดิม) ---
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            cv_depth = np.ascontiguousarray(cv_depth)
            o3d_depth = o3d.geometry.Image(cv_depth)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                o3d_depth, self.intrinsics, depth_scale=1000.0, depth_trunc=3.0, stride=2
            )

            if not pcd.has_points(): 
                # ถ้าไม่มีจุด ให้โชว์ภาพเปล่าๆ (ถ้ามี) แล้วออก
                if self.latest_color_img is not None:
                    cv2.imshow("Stair Detection Result", self.latest_color_img)
                    cv2.waitKey(1)
                return

            # Voxel Downsample & Align to 1000 points
            downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.08)
            points = np.asarray(downsampled_pcd.points)
            
            target_count = 1000
            current_count = len(points)
            if current_count == 0: return

            if current_count >= target_count:
                indices = np.random.choice(current_count, target_count, replace=False)
                final_points = points[indices]
            else:
                choice = np.random.choice(current_count, target_count - current_count, replace=True)
                final_points = np.concatenate([points, points[choice]])

            norm_points = self.normalize_point_cloud(final_points)

            # --- 2. Inference (AI Prediction - ส่วนเดิม) ---
            input_tensor = torch.from_numpy(norm_points).float().transpose(1, 0).unsqueeze(0).to(self.device)

            with torch.no_grad():
                outputs, _ = self.model(input_tensor)
                probs = torch.exp(outputs)
                confidence, predicted_idx = torch.max(probs, dim=1)
                
                pred_class = predicted_idx.item()
                conf_score = confidence.item() * 100

            # --- 3. แสดงผลด้วย OpenCV (ส่วนที่เพิ่มใหม่) ---
            
            # เตรียมภาพสำหรับการวาด (สร้าง copy เพื่อไม่ให้กระทบภาพต้นฉบับ)
            if self.latest_color_img is not None:
                display_img = self.latest_color_img.copy()
            else:
                # กรณีกล้องสีไม่ทำงาน สร้างภาพสีดำทดแทน
                display_img = np.zeros((self.intrinsics.height, self.intrinsics.width, 3), dtype=np.uint8)
                cv2.putText(display_img, "Waiting for color image...", (50, self.intrinsics.height // 2), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            result_text = f"{self.labels_map.get(pred_class, 'UNKNOWN')} ({conf_score:.1f}%)"
            box_color = self.label_colors.get(pred_class, (255, 255, 255))

            # วาด Label Box เฉพาะเมื่อเจอ Stair และ Confidence สูงพอ
            if pred_class in [0, 1]: # ถ้าเป็น Downstairs หรือ Upstairs
                if conf_score >= self.confidence_threshold:
                    self.draw_label_box(display_img, result_text, box_color)
                    # เปลี่ยนสี Log ใน Terminal ด้วย
                    if pred_class == 1: self.get_logger().warn(f">>> FOUND: {result_text} <<<")
                    else: self.get_logger().error(f">>> FOUND: {result_text} <<<")
                else:
                    # เจอแต่ Confidence ต่ำ วาดกล่องสีเทาแจ้งเตือน
                    self.draw_label_box(display_img, f"LOW CONF ({conf_score:.1f}%)", (128, 128, 128))
                    self.get_logger().info(f"Detected but low confidence: {result_text}")
            else:
                # กรณีเป็น Others (ไม่ใชบันได)
                self.get_logger().info(f"Scanned: {result_text}")
                # อาจจะเลือกไม่ต้องวาดอะไรเลย หรือวาดกล่องสีขาวบอกว่าไม่เจอ
                # self.draw_label_box(display_img, "NO STAIRS", box_color)

            # แสดงผลภาพในหน้าต่าง OpenCV
            cv2.imshow("Stair Detection Result", display_img)
            # สำคัญมาก! ต้องมี waitKey เพื่อให้ OpenCV อัปเดตหน้าต่าง
            cv2.waitKey(1) 

            # ส่ง Point Cloud ไปดูใน RViz (ส่วนเดิม)
            self.publish_point_cloud(norm_points, msg.header)

        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")

    def publish_point_cloud(self, points, header):
        # ... [โค้ดเดิมของคุณ] ...
        ros_msg = PointCloud2()
        ros_msg.header = header
        ros_msg.header.frame_id = "camera_depth_optical_frame"
        ros_msg.height = 1
        ros_msg.width = len(points)
        ros_msg.is_dense = False
        ros_msg.is_bigendian = False
        ros_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        ros_msg.point_step = 12
        ros_msg.row_step = ros_msg.point_step * points.shape[0]
        ros_msg.data = points.astype(np.float32).tobytes()
        self.pcd_pub.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StairInferenceNode()
    
    # ใช้ try-except เพื่อจัดการการปิดหน้าต่างอย่างสะอาด
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # ไม่ต้องใส่ cv2.waitKey ตรงนี้ เพราะมีอยู่ใน callback แล้ว
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        # ตรวจสอบว่า node ยังไม่ถูกทำลายก่อนสั่ง shutdown
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()