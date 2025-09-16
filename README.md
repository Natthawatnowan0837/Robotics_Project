# Robotics_Project

โปรเจกต์นี้เกี่ยวข้องกับ:
- ORB-SLAM (การสร้างแผนที่และระบุตำแหน่ง)
- Voice Speech (การสั่งงานด้วยเสียง)
- Intel RealSense D435i (Depth Camera)

---

## 🔹 การติดตั้ง Intel RealSense SDK (librealsense) บน Ubuntu 22.04

### 1. Gitclone librealsens for open Camera
```bash
sudo apt update
sudo apt install -y git cmake build-essential libssl-dev libusb-1.0-0-dev pkg-config \
libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

-การใช้งาน Intel Realsens d435i
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.55.1

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

realsense-viewer # ทดสอบ
