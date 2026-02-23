cmake_minimum_required(VERSION 3.8)
project(my_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# สร้าง Interfaces (Srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
  "srv/Conversation.srv"
  "srv/Sendposition.srv" 
  DEPENDENCIES std_msgs
)

# ---------------------------------------------------------
# 1. ติดตั้ง Python Node (Executables)
# ---------------------------------------------------------

# 1.1: เฉพาะ map_switcher.py ที่ต้องการเปลี่ยนชื่อ (ตัด .py ออก)
# เวลาใช้พิมพ์: ros2 run my_command_pkg map_switche

# 1.2: ไฟล์ Python อื่นๆ ที่ต้องการติดตั้งแบบชื่อเดิม
# เวลาใช้พิมพ์: ros2 run my_command_pkg simple_test.py


ament_package()