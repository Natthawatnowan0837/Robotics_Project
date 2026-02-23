#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import torch
import torch.nn as nn
import torch.nn.functional as F
import os

# ==========================================
# 1. นิยามโครงสร้างโมเดล (ต้องเหมือนตอน Train เป๊ะๆ)
# ==========================================
class TNet(nn.Module):
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
# 2. ROS 2 Node สำหรับ Inference
# ==========================================
class StairInferenceNode(Node):
    def __init__(self):
        super().__init__('stair_inference_node')
        
        self.bridge = CvBridge()
        self.intrinsics = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # --- โหลดโมเดล ---
        self.model_path = "pointnet_stairs3.pth" # ต้องวางไฟล์ไว้ที่เดียวกับที่รันคำสั่ง หรือใส่ path เต็ม
        
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file not found at {self.model_path}")
            # ถ้าหาไม่เจอ ให้ลองดู path ปัจจุบัน
            self.get_logger().error(f"Current path: {os.getcwd()}")
            return

        self.model = PointNet(classes=3).to(self.device)
        # Load weights (map_location สำคัญเผื่อ train บน GPU แต่ run บน CPU)
        self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
        self.model.eval() # สำคัญ! บอกโมเดลว่านี่คือโหมดใช้งาน ไม่ใช่โหมดฝึกสอน
        
        self.get_logger().info(f"Model Loaded Successfully! Device: {self.device}")
        
        # Class Label Mapping (ต้องเรียงตามที่ Train)
        # ปกติ alphabetical: 0=Downstairs, 1=Others, 2=Upstairs 
        # (หรือตรวจสอบจากตัวแปร classes ในโค้ด train ของคุณ)
        # สมมติว่าเป็น: {"downstairs": 0, "upstairs": 1, "others": 2}
        self.labels_map = {0: "DOWNSTAIRS", 1: "UPSTAIRS", 2: "OTHERS"} 

        # ROS Subscribers
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.info_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/inference_point_cloud', 10)

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic()
            self.intrinsics.set_intrinsics(msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def normalize_point_cloud(self, points):
        centroid = np.mean(points, axis=0)
        points_centered = points - centroid
        max_distance = np.max(np.linalg.norm(points_centered, axis=1))
        if max_distance == 0: max_distance = 1.0
        return points_centered / max_distance

    def depth_callback(self, msg):
        if self.intrinsics is None: return

        try:
            # 1. Preprocessing (เหมือนตอนเก็บข้อมูลเป๊ะๆ)
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            cv_depth = np.ascontiguousarray(cv_depth)
            o3d_depth = o3d.geometry.Image(cv_depth)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                o3d_depth, self.intrinsics, depth_scale=1000.0, depth_trunc=3.0, stride=2
            )

            if not pcd.has_points(): return

            # Downsample
            downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.08)
            points = np.asarray(downsampled_pcd.points)
            
            # Align to 1000 points
            target_count = 1000
            current_count = len(points)
            if current_count == 0: return

            if current_count >= target_count:
                indices = np.random.choice(current_count, target_count, replace=False)
                final_points = points[indices]
            else:
                choice = np.random.choice(current_count, target_count - current_count, replace=True)
                final_points = np.concatenate([points, points[choice]])

            # Normalize
            norm_points = self.normalize_point_cloud(final_points)

            # 2. Inference (AI Prediction)
            # แปลงเป็น Tensor: (1, 3, 1000)
            input_tensor = torch.from_numpy(norm_points).float()
            input_tensor = input_tensor.transpose(1, 0) # (1000,3) -> (3,1000)
            input_tensor = input_tensor.unsqueeze(0)    # Add batch dim -> (1,3,1000)
            input_tensor = input_tensor.to(self.device)

            with torch.no_grad(): # ไม่ต้องคำนวณ Gradient (เร็วขึ้น)
                outputs, _ = self.model(input_tensor)
                
                # outputs คือ Log Softmax -> แปลงเป็น Probability
                probs = torch.exp(outputs)
                confidence, predicted_idx = torch.max(probs, dim=1)
                
                pred_class = predicted_idx.item()
                conf_score = confidence.item() * 100

            # 3. แสดงผล
            result_text = f"{self.labels_map.get(pred_class, 'UNKNOWN')} ({conf_score:.1f}%)"
            
            # เปลี่ยนสี Log ตามผลลัพธ์เพื่อความตื่นเต้น
            if pred_class == 1: # Upstairs
                self.get_logger().warn(f">>> FOUND: {result_text} <<<") # สีเหลือง
            elif pred_class == 0: # Downstairs
                self.get_logger().error(f">>> FOUND: {result_text} <<<") # สีแดง
            else:
                self.get_logger().info(f"Scanned: {result_text}") # สีขาว

            # ส่ง Point Cloud ไปดูใน RViz
            self.publish_point_cloud(norm_points, msg.header)

        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")

    def publish_point_cloud(self, points, header):
        # (ฟังก์ชันเดิมสำหรับการ Visualize)
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()