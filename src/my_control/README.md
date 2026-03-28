# Robotics Project - Stair Robot with RTAB-Map SLAM

โปรเจกต์หุ่นยนต์เคลื่อนที่ (Stair Robot) โดยใช้ ROS 2 Humble ร่วมกับกล้อง RealSense D435i และ RTAB-Map สำหรับทำ SLAM และสร้างแผนที่ 3D

## 🛠 การเตรียมระบบ (Prerequisites)

ก่อนใช้งาน ตรวจสอบให้แน่ใจว่าได้ติดตั้ง Package สำคัญเหล่านี้แล้ว:
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [realsense2_camera](https://github.com/IntelRealSense/realsense-ros)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros)
- [imu_filter_madgwick](https://index.ros.org/p/imu_filter_madgwick/)

## 🚀 ขั้นตอนการติดตั้ง (Installation)

1. สร้าง Workspace และ Clone โปรเจกต์:
```bash
mkdir -p ~/Robotics_Project/src
cd ~/Robotics_Project/src
# Clone โปรเจกต์หลักของคุณ
git clone <URL_โปรเจกต์ของคุณ> .

# Clone rtabmap_ros (หากยังไม่มีในเครื่อง)
git clone [https://github.com/introlab/rtabmap_ros.git](https://github.com/introlab/rtabmap_ros.git)

คำสะั่งดูพิกัด

ros2 topic echo /rtabmap/localization_pose --once

คำสั่ง navigation 

ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=false \
params_file:=/home/noone/Robotics_Project/src/my_manager/config/nav2_params.yaml \
use_amcl:=false \
map:=/rtabmap/map \
