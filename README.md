# Robotics_Project

โปรเจกต์นี้เกี่ยวข้องกับ:
- ORB-SLAM (การสร้างแผนที่และระบุตำแหน่ง)
- Voice Speech (การสั่งงานด้วยเสียง)
- Intel RealSense D435i (Depth Camera)

---
## Install ROS2 Humble บน Ubuntu 22.04
สามารถทำตามคู่มือจาก ROS2 ได้ที่:  
[ROS2 Humble Ubuntu Development Setup](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

---
## Install Intel RealSense SDK (librealsense) บน Ubuntu 22.04

### 1. Git clone librealsens for open Camera
```bash
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.55.1

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

realsense-viewer # ทดสอบเปิด Software กล้อง

