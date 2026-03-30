#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException # เพิ่ม ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

# Import service interface
from my_command.srv import GotoStair 

class Go_to_stair(Node):
    def __init__(self):
        super().__init__('Go_to_stair')
        
        # --- [ ระบบ Service และ State Control ] ---
        self.is_active = False
        self.callback_group = ReentrantCallbackGroup()
        
        self.srv = self.create_service(
            GotoStair, 
            'goto_stair_service', 
            self.handle_goto_stair,
            callback_group=self.callback_group
        )

        # ... (ค่าตัวแปรอื่นๆ เหมือนเดิม) ...
        self.target_dist      = 0.45   
        self.min_vel          = 0.1   
        self.max_vel          = 0.5   
        self.min_angular      = 0.1   
        self.max_angular      = 0.5   
        self.angle_tolerance  = 20.0  
        self.search_ang_vel   = 0.7    
        self.conf_threshold   = 98.0
        self.max_range        = 4.5   
        self.ransac_iters     = 100   
        self.plane_thickness  = 0.04 
        self.roi_height_ratio = 0.6   
        self.lost_count = 0
        self.lost_threshold = 7  

        self.intrinsics = None
        qos_fast = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ปรับ Topic ให้ตรงตามที่คุณใช้งาน
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_rotation', 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.info_callback, 10)
        self.depth_sub = self.create_subscription(CompressedImage, '/camera/camera/depth/image_rect_raw/compressedDepth', self.depth_callback, qos_fast)
        
        self.get_logger().info("🚀 Stair Aligner Service Ready with Smooth Control.")

    def handle_goto_stair(self, request, response):
        if request.active:
            self.get_logger().info("📥 [SERVICE] Active Received: Starting Stair Alignment...")
            self.is_active = True
            self.lost_count = 0 
            response.success = True  # ตอบกลับทันทีว่าได้รับคำสั่งแล้ว
        else:
            self.get_logger().info("📥 [SERVICE] Deactivate Received.")
            self.is_active = False
            self.cmd_pub.publish(Twist()) # หยุดหุ่นยนต์
            response.success = True
        return response

    # ... (ฟังก์ชันคำนวณอื่นๆ เหมือนเดิม: info_callback, stair_alignment_control, depth_callback, etc.) ...
    def info_callback(self, msg):
        if self.intrinsics is None:
            self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            self.intrinsics = True

    def stair_alignment_control(self, dist, yaw_deg, confidence, detected):
        if not self.is_active: return
        cmd = Twist()
        if detected:
            if abs(yaw_deg) > self.angle_tolerance:
                target_ang_vel = -0.12 * yaw_deg 
                cmd.angular.z = np.sign(target_ang_vel) * max(self.min_angular, min(abs(target_ang_vel), self.max_angular))
            elif dist > self.target_dist:
                target_lin_vel = 0.25 * (dist - self.target_dist)
                cmd.linear.x = max(self.min_vel, min(target_lin_vel, self.max_vel))
            else:
                self.get_logger().info("🏁 [GOAL] Reached!")
                self.is_active = False
                self.cmd_pub.publish(Twist())
                return
        else:
            cmd.angular.z = self.search_ang_vel
        self.cmd_pub.publish(cmd)

    def depth_callback(self, msg):
        if not self.is_active or self.intrinsics is None: return
        try:
            depth_data = np.frombuffer(msg.data, np.uint8)
            cv_depth = cv2.imdecode(depth_data[12:], cv2.IMREAD_UNCHANGED)
            if cv_depth is None: return
            h, w = cv_depth.shape
            start_row = int(h * self.roi_height_ratio)
            cropped_depth = cv_depth[start_row:, :]
            stride = 10 
            reduced_depth = cropped_depth[::stride, ::stride]
            z = reduced_depth.astype(np.float32) / 1000.0
            mask = (z > 0.3) & (z < 5.0) 
            stair_found_this_frame = False
            if np.any(mask):
                v, u = np.indices(reduced_depth.shape)
                u_real, v_real = u[mask] * stride, (v[mask] * stride) + start_row
                z_valid = z[mask]
                x_cam = (u_real - self.cx) * z_valid / self.fx
                y_cam = (v_real - self.cy) * z_valid / self.fy
                points_link = np.zeros((len(z_valid), 3), dtype=np.float32)
                points_link[:, 0], points_link[:, 1], points_link[:, 2] = z_valid, -x_cam, -y_cam
                stair_data = self.extract_stair_features(points_link)
                if stair_data:
                    _, yaw_deg, confidence, dist = stair_data
                    if confidence >= self.conf_threshold:
                        stair_found_this_frame = True
                        self.lost_count = 0
                        self.stair_alignment_control(dist, yaw_deg, confidence, True)
            if not stair_found_this_frame:
                self.lost_count += 1
                if self.lost_count >= self.lost_threshold:
                    self.stair_alignment_control(0, 0, 0, False)
        except Exception as e:
            self.get_logger().error(f"Depth Error: {e}")

    def calculate_confidence(self, inliers_count, best_normal, width, dist):
        target_points = 200.0 if dist < 1.5 else 100.0
        point_score = min(inliers_count / target_points, 1.0) * 40
        alignment_score = abs(best_normal[0]) * 45
        width_score = min(width / 0.5, 1.0) * 15
        return point_score + alignment_score + width_score

    def extract_stair_features(self, points):
        best_inliers, best_normal = [], None
        mask_3d = (points[:, 0] > 0.1) & (points[:, 0] < self.max_range) & \
                  (points[:, 1] > -0.75) & (points[:, 1] < 0.75) & \
                  (points[:, 2] > -0.35)
        p_filtered = points[mask_3d]
        if len(p_filtered) < 25: return None
        for _ in range(self.ransac_iters):
            idx = np.random.choice(len(p_filtered), 3, replace=False)
            p1, p2, p3 = p_filtered[idx]
            normal = np.cross(p2-p1, p3-p1)
            n_norm = np.linalg.norm(normal)
            if n_norm < 1e-6: continue
            normal /= n_norm
            if normal[0] > 0: normal = -normal 
            if abs(normal[0]) < 0.85: continue
            d = -np.dot(normal, p1)
            inliers = np.where(np.abs(np.dot(p_filtered, normal) + d) < self.plane_thickness)[0]
            if len(inliers) > len(best_inliers):
                best_inliers, best_normal = inliers, normal
        if len(best_inliers) > 35:
            stair_pts = p_filtered[best_inliers]
            width = np.max(stair_pts[:, 1]) - np.min(stair_pts[:, 1])
            if width > 0.25:
                yaw_deg = math.degrees(math.atan2(best_normal[1], -best_normal[0]))
                dist = np.mean(stair_pts[:, 0])
                confidence = self.calculate_confidence(len(best_inliers), best_normal, width, dist)
                return stair_pts, yaw_deg, confidence, dist
        return None

# --- [ ส่วนการปิดที่สมบูรณ์ ] ---
def main(args=None):
    rclpy.init(args=args)
    
    node = Go_to_stair()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('🛑 Shutting down stair alignment...')
    finally:
        # สำคัญ: ต้องหยุดหุ่นยนต์ก่อนทำลายโหนด
        node.is_active = False
        
        # สร้างพัปลิชเชอร์สำรองสั้นๆ หรือใช้ตัวเดิมส่งค่า 0 เพื่อหยุดทันที
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.get_logger().info('👋 Robot stopped.')

        # ลำดับการปิดทรัพยากร
        executor.shutdown()
        node.destroy_node()
        
        # ตรวจสอบสถานะก่อน shutdown เพื่อป้องกัน Error ซ้ำซ้อน
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()