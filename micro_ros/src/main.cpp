#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "main.h"
#define LED_PIN 2
// --- ตัวแปรระดับ Global ---
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_publisher_t pub_drive,pub_statePlatform,pub_stateArm,pub_balance;
rcl_subscription_t sub_motor,sub_controller,sub_sensors,sub_pid;

std_msgs__msg__Float32 msg_pub_stateArm;
std_msgs__msg__Float32MultiArray msg_pub_drive,msg_pub_statePlatform,msg_pub_balance; // สำหรับส่งออก
std_msgs__msg__Float32MultiArray msg_sub_motor,msg_sub_controller,msg_sub_sensors,msg_sub_pid; // สำหรับรับเข้า

// float pub_buffer[5]; // จองพื้นที่ส่ง (ปรับจำนวนได้)
float motor_data[4]; // จองพื้นที่รับ (ปรับจำนวนได้)
float controller_data[4];
float sensors_data[7];
float pid_data[12];
//-------------------
float drive_report[4];
float statePlatform_report[2];
float balance_report[4];

//------------------------
float motorDrive_L = 0.0; 
float motorDrive_R = 0.0;
float motorArm_L = 0.0;
float motorArm_R = 0.0;
//------------------------
float linear_control = 0.0; 
float angular_control = 0.0;
float platform_control = 0.0;
float arm_control = 0.0;
//-------------------------
float body_x = 0.0;
float body_y = 0.0 ;
float platform_x = 0.0;
float platform_y = 0.0; 
float hall_effect = 0.0;
float omega_body_y = 0.0;
float omega_platform_y = 0.0;
//------------------------
float pid_driveL_parameters[3];
float pid_driveR_parameters[3];
float pid_platform_parameters[3];
float pid_arm_parameters[3];

// --- Callback: เมื่อได้รับข้อมูลจาก ROS 2 ---
void motor_callback(const void * msgin) {
  // 1. Cast ข้อมูลที่รับเข้ามาให้เป็นชนิด Float32MultiArray
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 4) {
    motorDrive_L = msg->data.data[0];
    motorDrive_R = msg->data.data[1];
    motorArm_L   = msg->data.data[2];
    motorArm_R   = msg->data.data[3];
  }
}

void controller_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 4) {
    linear_control   = msg->data.data[0];
    angular_control  = msg->data.data[1];
    platform_control = msg->data.data[2];
    arm_control      = msg->data.data[3]; 
  }
}

void sensors_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  body_x = sensors_data[0];
  body_y = sensors_data[1];
  platform_x = sensors_data[2];
  platform_y = sensors_data[3]; 
  hall_effect = sensors_data[4];
  omega_body_y = sensors_data[5];
  omega_platform_y == sensors_data[6];
}

void pid_callback(const void * msgin) {
  // 1. Cast ข้อมูลจาก void* เป็น Float32MultiArray
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // 2. ตรวจสอบว่าข้อมูลมาครบ (12 ค่าตามที่ตั้งไว้ใน Python)
  if (msg->data.size >= 12) {
    // PID ล้อซ้าย (Index 0, 1, 2)
    pid_driveL_parameters[0] = msg->data.data[0]; // Kp
    pid_driveL_parameters[1] = msg->data.data[1]; // Ki
    pid_driveL_parameters[2] = msg->data.data[2]; // Kd

    // PID ล้อขวา (Index 3, 4, 5)
    pid_driveR_parameters[0] = msg->data.data[3]; // Kp
    pid_driveR_parameters[1] = msg->data.data[4]; // Ki
    pid_driveR_parameters[2] = msg->data.data[5]; // Kd

    // PID Platform (Index 6, 7, 8)
    pid_platform_parameters[0] = msg->data.data[6];
    pid_platform_parameters[1] = msg->data.data[7];
    pid_platform_parameters[2] = msg->data.data[8];

    // PID Arm (Index 9, 10, 11)
    pid_arm_parameters[0] = msg->data.data[9];
    pid_arm_parameters[1] = msg->data.data[10];
    pid_arm_parameters[2] = msg->data.data[11];
  }
}
// --- Timer: สำหรับส่งข้อมูลออก (ทำงานทุก 20ms หรือ 50Hz) ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    pid_drive(linear_control,angular_control,motorDrive_L,motorDrive_R);
    pid_plateform(platform_y,hall_effect,omega_platform_y);
    // pwm_motor(linear_control,angular_control),pid_drive[3];
    // pwm_platform(platform_control,hall_effect);
    // pwm_arm(arm_control);
    // หยอดข้อมูลลง pub_data[i] ก่อนส่ง
    // RCSOFTCHECK(rcl_publish(&pub_data, &msg_pub, NULL));
  }
}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setup() {
  init_PID();
  init_plateformPID();
  set_microros_transports(); // เริ่มต้น Serial Transport
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_base_node", "", &support));

  // --- 1. เคลียร์และจอง Memory สำหรับ MultiArray (สำคัญที่สุด) ---งจริง

  msg_sub_motor.data.data = motor_data;
  msg_sub_motor.data.capacity = 4;
  
  msg_sub_controller.data.data = controller_data;
  msg_sub_controller.data.capacity = 4;

  msg_sub_sensors.data.data = sensors_data;
  msg_sub_sensors.data.capacity = 7;

  msg_sub_pid.data.data = pid_data;
  msg_sub_pid.data.capacity = 12;
  
  //---------------------------------------------
  msg_pub_drive.data.data = drive_report;
  msg_pub_drive.data.capacity = 4;
  msg_pub_drive.data.size = 4;
  msg_pub_drive.layout.dim.capacity = 0;
  msg_pub_drive.layout.dim.size = 0;
  msg_pub_drive.layout.data_offset = 0;

  msg_pub_statePlatform.data.data = statePlatform_report;
  msg_pub_statePlatform.data.capacity = 2;
  msg_pub_statePlatform.data.size = 2;
  msg_pub_statePlatform.layout.dim.capacity = 0;
  msg_pub_statePlatform.layout.dim.size = 0;
  msg_pub_statePlatform.layout.data_offset = 0;

  msg_pub_balance.data.data = balance_report;
  msg_pub_balance.data.capacity = 4;
  msg_pub_balance.data.size = 4;
  msg_pub_balance.layout.dim.capacity = 0;
  msg_pub_balance.layout.dim.size = 0;
  msg_pub_balance.layout.data_offset = 0;

  // --- 2. เริ่มต้น Publisher และ Subscription ---
  RCCHECK(rclc_publisher_init_default(&pub_drive, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "stageDrive"));
  RCCHECK(rclc_publisher_init_default(&pub_statePlatform, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "stagePlatform"));  
  RCCHECK(rclc_publisher_init_default(&pub_balance, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "balance"));  
  RCCHECK(rclc_publisher_init_default(&pub_stateArm, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "stageArm"));  

  //----------------------------------------------
  RCCHECK(rclc_subscription_init_default(&sub_motor, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motors"));
  RCCHECK(rclc_subscription_init_default(&sub_sensors, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "sensors"));
  RCCHECK(rclc_subscription_init_default(&sub_controller, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "controller"));
  RCCHECK(rclc_subscription_init_default(&sub_pid, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "pid_parameters"));

  // --- 3. เริ่มต้น Timer และ Executor ---
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));

  // Executor รองรับ 2 งาน: 1 Subscription + 1 Timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor, &msg_sub_motor, &motor_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_controller, &msg_sub_controller, &controller_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_sensors, &msg_sub_sensors, &sensors_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_pid, &msg_sub_pid, &pid_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  // สั่งให้ระบบทำงาน (Spin)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}