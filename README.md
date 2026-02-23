<!-- Mapping Loading -->

ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu \
    qos:=1 \
    rviz:=true

<!-- Localization -->
ros2 launch rtabmap_launch rtabmap.launch.py \
    localization:=true \
    database_path:="~/.ros/rtabmap.db" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu \
    qos:=1 \
    rviz:=true