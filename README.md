# RTAB_Map
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=true \
  qos:=2 \
  rtabmap_viz:=true \
  rviz:=true \
  localization:=true \
  rtabmap_args:="--Mem/IncrementalMemory false"


ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  pointcloud.enable:=true \
  depth_module.profile:=848x480x30 \
  rgb_camera.profile:=848x480x30 \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2


export TURTLEBOT3_MODEL=burger #start
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

export TURTLEBOT3_MODEL=burger #sim
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
use_sim_time:=true \
map:=/home/jo/RTAB_Map/src/my_command_pkg/maps/floor1/rtabmap2.yaml


killall -9 component_container_isolated


colcon build --parallel-workers 1

รับ point cloud (จาก RealSense หรือไฟล์)

Filter / downsample

หา floor plane + คำนวณ transform camera↔floor

หา planes / obstacles ทั้งฉาก

จัดประเภท plane + หาทิศทางหลัก (Manhattan)

วิเคราะห์ว่ามี stair หรือเปล่า → ถ้ามี, model + visualize

วนไปทีละเฟรม