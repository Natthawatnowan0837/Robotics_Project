import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import io

# --- Config Matplotlib (Headless) ---
import matplotlib
matplotlib.use('Agg') # ใช้โหมดวาดลง Memory เพื่อกันโปรแกรมค้าง
import matplotlib.pyplot as plt

class StairProfileCV(Node):
    def __init__(self):
        super().__init__('stair_profile_cv')
        
        # Parameters
        self.declare_parameter('strip_width', 40)
        self.strip_width = self.get_parameter('strip_width').value
        
        # Topic (เช็คชื่อให้ตรงกับเครื่องคุณ)
        target_topic = '/camera/camera/depth/image_rect_raw'
        
        self.subscription = self.create_subscription(
            Image, target_topic, self.listener_callback, 10)
            
        self.bridge = CvBridge()
        self.get_logger().info(f"Stair Profile (CV Window) Started. Watching: {target_topic}")

    def listener_callback(self, msg):
        try:
            # 1. แปลงภาพจาก ROS เป็น OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 2. แปลงหน่วยเป็นเมตร (D435i ค่าดิบคือ mm)
            depth_meters = cv_image.astype(np.float32) * 0.001
            depth_meters[depth_meters == 0] = np.nan # เปลี่ยน 0 เป็น NaN
            
            # 3. ตัดแถบกลางภาพ (Center Strip)
            height, width = depth_meters.shape
            center_x = width // 2
            half_strip = self.strip_width // 2
            strip = depth_meters[:, center_x - half_strip : center_x + half_strip]
            
            # ถ้ามืดสนิท (ไม่มีข้อมูล) ให้ข้าม
            if np.all(np.isnan(strip)):
                return
            
            # 4. สร้างกราฟ Profile (หาค่าเฉลี่ยแนวนอน)
            profile = np.nanmean(strip, axis=1)
            profile = profile[::-1] # กลับด้าน (ล่างขึ้นบน)
            
            # 5. วาดกราฟลงภาพ
            graph_image = self.draw_graph(profile)
            
            # 6. แสดงผลผ่านหน้าต่าง OpenCV (GUI)
            cv2.imshow("Stair Depth Profile", graph_image)
            cv2.waitKey(1) # จำเป็นต้องมีเพื่อให้หน้าต่างอัปเดต

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def draw_graph(self, profile):
        # ตั้งค่ากราฟ
        fig, ax = plt.subplots(figsize=(6, 4), dpi=80)
        
        x = np.arange(len(profile))
        
        # วาดเส้นกราฟ
        ax.plot(x, profile, label='Depth', color='blue', linewidth=2)
        
        # ตกแต่งกราฟ
        ax.set_title(f"Stair Profile (Center Strip: {self.strip_width}px)")
        ax.set_xlabel("Pixel Height (Bottom -> Top)")
        ax.set_ylabel("Depth (Meters)")
        ax.set_ylim(0, 3.0) # ฟิกแกน Y ที่ 0-3 เมตร (ปรับได้)
        ax.grid(True, linestyle='--', alpha=0.6)
        
        # --- แปลง Matplotlib Figure -> OpenCV Image ---
        buf = io.BytesIO()
        plt.savefig(buf, format='png') # บันทึกลง Ram
        buf.seek(0)
        
        # อ่านข้อมูลจาก Buffer เป็น Array
        img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        buf.close()
        plt.close(fig) # ปิด Figure เพื่อคืน Ram ทันที
        
        # Decode เป็นภาพ OpenCV
        img = cv2.imdecode(img_arr, 1)
        return img

def main(args=None):
    rclpy.init(args=args)
    node = StairProfileCV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows() # ปิดหน้าต่างเมื่อกด Ctrl+C
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()