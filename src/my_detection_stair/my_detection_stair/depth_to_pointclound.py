#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import open3d as o3d
import os
import time

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        
        self.bridge = CvBridge()
        self.intrinsics = None

        # --- 1. ตั้งค่าการบันทึกข้อมูล (Configuration) ---
        # ประกาศ Parameter เพื่อให้เปลี่ยนชื่อ Class ผ่าน Command Line ได้
        self.declare_parameter('class_label', 'others') 
        self.class_label = self.get_parameter('class_label').get_parameter_value().string_value
        
        # ตั้งค่าโฟลเดอร์ที่จะบันทึก
        self.base_dir = "dataset_stair"
        self.save_dir = os.path.join(self.base_dir, self.class_label)
        
        # สร้างโฟลเดอร์ถ้ายังไม่มี
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Created directory: {self.save_dir}")
        else:
            self.get_logger().info(f"Saving data to: {self.save_dir}")

        # ตัวแปรสำหรับหน่วงเวลาบันทึก (ไม่ให้รัวเกินไป)
        self.last_save_time = time.time()
        self.save_interval = 0.5  # บันทึกทุกๆ 0.5 วินาที (ปรับได้)
        self.file_count = len(os.listdir(self.save_dir)) # เริ่มนับต่อจากไฟล์เดิม

        # --- 2. ROS Subscribers ---
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.info_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Publisher สำหรับดูผลลัพธ์ใน RViz (Optional)
        self.pcd_pub = self.create_publisher(PointCloud2, '/preview_collection', 10)
        
        self.get_logger().info(f"--- READY TO COLLECT: {self.class_label.upper()} ---")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic()
            self.intrinsics.set_intrinsics(
                width=msg.width,
                height=msg.height,
                fx=msg.k[0], fy=msg.k[4],
                cx=msg.k[2], cy=msg.k[5]
            )

    def normalize_point_cloud(self, points):
        # ทำตามสมการที่ (1) ใน Paper
        centroid = np.mean(points, axis=0)
        points_centered = points - centroid
        distances = np.linalg.norm(points_centered, axis=1)
        max_distance = np.max(distances)
        if max_distance == 0: max_distance = 1.0
        return points_centered / max_distance

    def depth_callback(self, msg):
        if self.intrinsics is None:
            return

        try:
            # 1. Convert ROS -> OpenCV -> Open3D
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            cv_depth = np.ascontiguousarray(cv_depth)
            o3d_depth = o3d.geometry.Image(cv_depth)
            
            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                o3d_depth, 
                self.intrinsics,
                depth_scale=1000.0,
                depth_trunc=3.0,
                stride=2 # เพิ่ม stride เพื่อให้เร็วขึ้นเล็กน้อย
            )

            if not pcd.has_points(): return

            # 2. Downsampling & Align to 1000 points
            # ใช้ค่า voxel ใหญ่หน่อยเพื่อให้จุดเหลือน้อยเร็วๆ
            downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.08) 
            points = np.asarray(downsampled_pcd.points)
            
            current_count = len(points)
            target_count = 1000
            
            if current_count == 0: return

            if current_count >= target_count:
                indices = np.random.choice(current_count, target_count, replace=False)
                final_points = points[indices]
            else:
                # Padding (สุ่มจุดเดิมมาเติมให้ครบ)
                choice = np.random.choice(current_count, target_count - current_count, replace=True)
                final_points = np.concatenate([points, points[choice]])

            # 3. Normalization (สำคัญมาก! ต้องบันทึกค่าที่ Normalize แล้ว)
            norm_points = self.normalize_point_cloud(final_points)

            # 4. Save Data (.npy)
            # เช็คเวลาว่าผ่านไปเกิน 0.5 วินาทีหรือยัง
            if time.time() - self.last_save_time > self.save_interval:
                filename = f"{self.class_label}_{self.file_count:05d}.npy"
                full_path = os.path.join(self.save_dir, filename)
                
                # บันทึกไฟล์
                np.save(full_path, norm_points)
                
                self.get_logger().info(f"Saved: {filename} (Total: {self.file_count + 1})")
                self.file_count += 1
                self.last_save_time = time.time()
                
                # ส่งไปดูใน RViz เพื่อความชัวร์ว่าภาพที่บันทึกดูรู้เรื่อง
                self.publish_point_cloud(norm_points, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def publish_point_cloud(self, points, header):
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
    node = DataCollectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()