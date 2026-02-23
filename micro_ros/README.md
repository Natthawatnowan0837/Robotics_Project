ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200


export ROS_DOMAIN_ID=0
ros2 daemon stop
ros2 daemon start
ros2 topic list