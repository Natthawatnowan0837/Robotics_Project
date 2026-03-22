cmake_minimum_required(VERSION 3.8)
project(my_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# 1. FIND DEPENDENCIES
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED) # <--- จำเป็นมากสำหรับ .action
find_package(rosidl_default_generators REQUIRED)

# ---------------------------------------------------------
# 2. GENERATE INTERFACES (Srv & Action)
# ---------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
  "srv/Conversation.srv"
  "srv/Sendposition.srv" 
  "srv/CheckFloor.srv"
  "action/ExecuteCommand.action" # <--- ย้ายมาอยู่ในโฟลเดอร์ action/ ตามคอนเวนชั่น
  DEPENDENCIES std_msgs action_msgs # <--- เพิ่ม action_msgs ตรงนี้ด้วย
)

# ---------------------------------------------------------
# 3. INSTALL PYTHON SCRIPTS
# ---------------------------------------------------------
# สำหรับ ROS 2 ที่เป็น ament_cmake แต่มีไฟล์ python 
# แนะนำให้ติดตั้งลงใน lib ของ package เพื่อให้ ros2 run หาเจอ

# ---------------------------------------------------------
# 4. AMENT PACKAGE
# ---------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()