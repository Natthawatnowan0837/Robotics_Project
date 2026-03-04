#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#define LED_PIN 2
#include "main.h"

// --- Micro-ROS objects ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Subscription & Publishers
rcl_subscription_t sub_motor_rps, sub_cmd_vel , sub_arm_deg ,sub_platform_vel,sub_hall_effect,
                    sub_gyro_body,sub_gyro_platform;
rcl_publisher_t pub_rps_l, pub_rps_r, pub_deg_l, pub_deg_r , pub_vel_out, pub_setpoint ,pub_platform_vel_out;

// Messages
std_msgs__msg__Float32MultiArray msg_sub_rps , msg_arm_deg , msg_vel_out , msg_setpoint , msg_platform_vel_out;
std_msgs__msg__Float32 msg_rpsl, msg_rpsr, msg_degl, msg_degr , msg_gyro_body, msg_gyro_platform;
std_msgs__msg__Bool msg_hall_effect;
geometry_msgs__msg__Twist msg_cmd_vel , msg_platform_vel;

// จอง Memory สำหรับรับข้อมูล Array (ต้องจองไว้ล่วงหน้า)
static float rps_buffer[5]; 
static float arm_buffer[2];
static float vel_out_buffer[2];
static float setpoint_buffer[2];
static float platform_vel_out_buffer[2];

float current_linear = 0.0, current_angular = 0.0;
float rps_l = 0.0, rps_r = 0.0;
float deg_l = 0.0, deg_r = 0.0;
bool limited = false;
unsigned long last_cmd_vel_time = 0; // เก็บเวลาที่ได้รับข้อความล่าสุด
// const unsigned long CMD_VEL_TIMEOUT = 500; // ตัดการทำงานหากไม่ได้รับค่าเกิน 0.5 วินาที
// --- Error Handling ---
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  last_cmd_vel_time = millis();
  
  current_linear  = msg->linear.x;
  current_angular = msg->angular.z;

  // pid_motor(current_linear, current_angular, rps_l, rps_r);
  // pwm_(current_linear, current_angular);
}

void hall_effect_callback(const void * msgin) {
  // แปลงข้อมูลที่รับเข้ามาให้เป็นชนิด Bool message
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  
  // เก็บค่า True/False ลงในตัวแปรสำหรับใช้งานในเงื่อนไขอื่นๆ
  limited = msg->data; 
  
  // อัปเดตค่าในตัวแปร message เพื่อใช้สำหรับ Publish (ถ้าจำเป็น)
  msg_hall_effect.data = msg->data; 
}

void gyro_body_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  msg_gyro_body.data = msg->data; 
}

void gyro_platform_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  msg_gyro_platform.data = msg->data; 
}


void platform_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  last_cmd_vel_time = millis();
  current_linear = msg->linear.x;

  // ส่งค่า limited ที่ได้จาก Hall Effect เข้าไปในฟังก์ชันควบคุมมอเตอร์
  pwm_platform(current_linear, limited, msg_gyro_body.data, msg_gyro_platform.data);
}

// void check_cmd_timeout() {
//   if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT) {
//     // หากเกินเวลาที่กำหนด ให้สั่งหยุดมอเตอร์เพื่อความปลอดภัย
//     pwm(0.0, 0.0); 
//   }
// }

// --- Callback Function ---
void rps_callback(const void * msgin) {
  // เมื่อมีข้อมูลเข้า ให้ไฟสถานะเปลี่ยนค่า (Toggle)
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // ตรวจสอบว่ามีข้อมูลส่งมาอย่างน้อย 2 ค่า
  if (msg->data.size >= 2) {
    // 1. ดึงค่าออกมาและปัดเศษให้เหลือ 2 ตำแหน่งทันที
    // สูตร: round(ค่า * 100) / 100
    rps_l = roundf(msg->data.data[0] * 100.0f) / 100.0f;
    rps_r = roundf(msg->data.data[1] * 100.0f) / 100.0f;

    // 2. จัดการเรื่อง Deadzone (ถ้าค่าน้อยมากๆ ให้เป็น 0.00)
    if (abs(rps_l) < 0.01f) rps_l = 0.00f;
    if (abs(rps_r) < 0.01f) rps_r = 0.00f;

    // 3. ส่งค่าออกไปที่ Topic Debug
    msg_rpsl.data = rps_l;
    msg_rpsr.data = rps_r;
    
    RCSOFTCHECK(rcl_publish(&pub_rps_l, &msg_rpsl, NULL));
    RCSOFTCHECK(rcl_publish(&pub_rps_r, &msg_rpsr, NULL));
  }
}

void arm_callback(const void * msgin) {
  // เมื่อมีข้อมูลเข้า ให้ไฟสถานะเปลี่ยนค่า (Toggle)
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // ตรวจสอบว่ามีข้อมูลส่งมาอย่างน้อย 2 ค่า
  if (msg->data.size >= 2) {
    // 1. ดึงค่าออกมา
    deg_l = msg->data.data[0];
    deg_r = msg->data.data[1];

    // 2. ส่งค่าออกไปที่ Topic Debug
    msg_degl.data = deg_l;
    msg_degr.data = deg_r;
    
    RCSOFTCHECK(rcl_publish(&pub_deg_l, &msg_degl, NULL));
    RCSOFTCHECK(rcl_publish(&pub_deg_r, &msg_degr, NULL));
  }
}

void setup() {
  // init_PID();
  pinMode(LED_PIN, OUTPUT);
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // ตั้งชื่อ Node ให้ต่างจากตัวแรก
  RCCHECK(rclc_node_init_default(&node, "esp32_debug_sub_node", "", &support));

  // --- 1. เตรียม Memory สำหรับ Message ที่จะ Subscribe ---
  msg_sub_rps.data.data = rps_buffer;
  msg_sub_rps.data.capacity = 5;
  msg_sub_rps.data.size = 0;

  msg_arm_deg.data.data = arm_buffer;
  msg_arm_deg.data.capacity = 2;
  msg_arm_deg.data.size = 0;

  msg_vel_out.data.data = vel_out_buffer;
  msg_vel_out.data.capacity = 2;
  msg_vel_out.data.size = 2;

  msg_setpoint.data.data = setpoint_buffer;
  msg_setpoint.data.capacity = 2;
  msg_setpoint.data.size = 2;

  msg_platform_vel_out.data.data = platform_vel_out_buffer;
  msg_platform_vel_out.data.capacity = 2;
  msg_platform_vel_out.data.size = 2;

  // --- 2. Init Publishers (สำหรับ Debug) ---
 
  RCCHECK(rclc_publisher_init_default(&pub_vel_out, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "wheel_velocity_output"));
  RCCHECK(rclc_publisher_init_default(&pub_setpoint, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "wheel_setpoint"));
  RCCHECK(rclc_publisher_init_default(&pub_platform_vel_out, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "platform_velocity_output"));
  
  // --- 3. Init Subscriber ---
  RCCHECK(rclc_subscription_init_default(&sub_motor_rps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motor_rps_array"));
  RCCHECK(rclc_subscription_init_default(&sub_arm_deg, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "arm_deg_array"));  
  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&sub_platform_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "platform_cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&sub_hall_effect, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "hall_effect"));
  RCCHECK(rclc_subscription_init_default(&sub_gyro_body, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "gyro_body"));
  RCCHECK(rclc_subscription_init_default(&sub_gyro_platform, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "gyro_platform"));
  // เปิดไฟค้างไว้เมื่อพร้อมทำงาน
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_rps, &msg_sub_rps, &rps_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_arm_deg, &msg_arm_deg, &arm_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_platform_vel, &msg_platform_vel, &platform_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_hall_effect, &msg_hall_effect, &hall_effect_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_gyro_body, &msg_gyro_body, &gyro_body_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_gyro_platform, &msg_gyro_platform, &gyro_platform_callback, ON_NEW_DATA));
  
  // เปิดไฟค้างไว้เมื่อพร้อมทำงาน
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // check_cmd_timeout();
  // สั่งให้ Executor ทำงาน
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}