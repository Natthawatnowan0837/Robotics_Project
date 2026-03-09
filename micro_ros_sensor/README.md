# =========================================================
# micro-ROS Setup Guide (ROS 2 + Serial)
# =========================================================

# -----------------------------
# 1) สร้าง ROS 2 Workspace
# -----------------------------
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src


# -----------------------------
# 2) Clone micro-ROS Packages
# -----------------------------
git clone https://github.com/micro-ROS/micro-ROS-Agent.git
git clone https://github.com/micro-ROS/micro_ros_msgs.git

# ตรวจสอบโครงสร้าง
# ros2_ws/
#  └── src/
#      ├── micro-ROS-Agent/
#      └── micro_ros_msgs/


# -----------------------------
# 3) Build Workspace
# -----------------------------
cd ~/ros2_ws
colcon build
source install/setup.bash

# หมายเหตุ:
# ทุกครั้งที่เปิด Terminal ใหม่ ต้องรัน:
# source ~/ros2_ws/install/setup.bash


# -----------------------------
# 4) ตรวจสอบ Serial Port
# -----------------------------
ls /dev/ttyUSB*

# ตัวอย่างผลลัพธ์:
# /dev/ttyUSB0

# ถ้า permission ไม่พอ:
sudo usermod -a -G dialout $USER

# จากนั้น logout/login ใหม่ หรือ reboot


# -----------------------------
# 5) ตั้งค่า ROS Domain ID
# -----------------------------
export ROS_DOMAIN_ID=0

# ต้องตั้งค่าให้ตรงกันทั้ง Agent และ microcontroller


# -----------------------------
# 6) Restart ROS 2 Daemon
# -----------------------------
ros2 daemon stop
ros2 daemon start


# -----------------------------
# 7) รัน micro-ROS Agent (Serial Mode)
# -----------------------------
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# อธิบาย:
# serial              = ใช้งานผ่าน Serial
# --dev /dev/ttyUSB0  = ระบุพอร์ต
# -b 115200           = baudrate ต้องตรงกับฝั่งบอร์ด

36
# =========================================================
# เปิดอีก Terminal สำหรับตรวจสอบ Topic
# =========================================================

source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 topic list

# ถ้าเชื่อมต่อสำเร็จ จะเห็น topic จาก microcontroller


# ทดสอบ echo topic
ros2 topic echo /your_topic_name


# =========================================================
# Troubleshooting
# =========================================================
# - ตรวจสอบ baudrate ให้ตรงกัน
# - ตรวจสอบว่า /dev/ttyUSB0 ถูกต้อง
# - เช็ค ROS_DOMAIN_ID ให้ตรงกัน
# - Restart daemon ใหม่
# - ตรวจสอบ firmware ฝั่ง microcontroller
# =========================================================