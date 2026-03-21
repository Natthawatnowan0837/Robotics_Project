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
rcl_subscription_t sub_motor,sub_arm_vel,sub_sensors,sub_pid,
                    sub_cmd_vel;

std_msgs__msg__Float32 msg_pub_stateArm;
std_msgs__msg__Float32MultiArray msg_pub_drive,msg_pub_statePlatform,msg_pub_balance; // สำหรับส่งออก
std_msgs__msg__Float32MultiArray msg_sub_motor,msg_sub_sensors,msg_sub_pid; // สำหรับรับเข้า
geometry_msgs__msg__Twist msg_cmd_vel,msg_sub_arm_vel;

float motor_data[8]; // จองพื้นที่รับ (ปรับจำนวนได้)
float sensors_data[11];
float pid_data[12];
//-------------------
float drive_report[4];
float statePlatform_report[2];
float balance_report[3];

//------------------------
float motorDrive_L = 0.0f; 
float motorDrive_R = 0.0f;
float motorArm_L = 0.0f;
float motorArm_R = 0.0f;
//------------------------
float linear_arm = 0.0; 
float platform_control = 0.0;
//-------------------------
float current_linear_x = 0.0f;
float current_angular_z = 0.0f;
//-------------------------
// --- [ ข้อมูล BODY สำหรับ EKF / RTAB-Map ] ---
float fAccelX = 0.0f;    // Index 0: ความเร่งแกน X (m/s^2) - ผ่าน Filter
float fAccelY = 0.0f;    // Index 1: ความเร่งแกน Y (m/s^2) - ผ่าน Filter
float fAccelZ = 0.0f;    // Index 2: ความเร่งแกน Z (m/s^2) - ผ่าน Filter (ปกติ ~9.8)

float body_gyro_x = 0.0f; // Index 3: ความเร็วเชิงมุมแกน X (rad/s) - ลบ Bias แล้ว
float body_gyro_y = 0.0f; // Index 4: ความเร็วเชิงมุมแกน Y (rad/s) - ลบ Bias แล้ว
float body_gyro_z = 0.0f; // Index 5: ความเร็วเชิงมุมแกน Z (rad/s) - สำหรับ Yaw Fusion

// --- [ ข้อมูล PLATFORM สำหรับควบคุม / แสดงผล ] ---
float anglePlatformX = 0.0f; // Index 6: มุมเอียง X (deg) - ผ่าน Filter
float anglePlatformY = 0.0f; // Index 7: มุมเอียง Y (deg) - ผ่าน Filter

// --- [ ข้อมูลเสริม (ถ้ามี) ] ---
float hall_effect = 0.0f;    // Index 8: (หากต้องการส่งค่าความเร็วจาก Encoder เพิ่ม)

//------------------------
float pid_driveL_parameters[3];
float pid_driveR_parameters[3];
float pid_platform_parameters[3];
float pid_arm_parameters[3];

// --- Callback: เมื่อได้รับข้อมูลจาก ROS 2 ---
void motor_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 8) {
    motorDrive_L = msg->data.data[0];
    motorDrive_R = msg->data.data[1];
    motorArm_L   = msg->data.data[6];
    motorArm_R   = msg->data.data[7];
  }
}


void sensors_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // ตรวจสอบว่าข้อมูลส่งมาอย่างน้อย 9 ตัว (Index 0-8)
  if (msg->data.size >= 9) {
    float * sensors_data = msg->data.data;

    fAccelX     = sensors_data[0];
    fAccelY     = sensors_data[1];
    fAccelZ     = sensors_data[2];
    body_gyro_x = sensors_data[3];
    body_gyro_y = sensors_data[4];
    body_gyro_z = sensors_data[5];
    anglePlatformX = sensors_data[6];
    anglePlatformY = sensors_data[7];
    hall_effect    = sensors_data[8];
  }
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

void cmd_vel_callback(const void * msgin) {
  // 1. Cast ข้อมูลจาก void pointer เป็น Twist message
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // 2. อัปเดตค่าลงใน Global Variable (ห้ามใส่คำว่า float ข้างหน้า)
  current_linear_x  = (float)msg->linear.x;  
  current_angular_z = (float)msg->angular.z; 
}

void arm_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  linear_arm  = (float)msg->linear.x;  
}

// --- Timer: สำหรับส่งข้อมูลออก (ทำงานทุก 20ms หรือ 50Hz) ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    // pid_drive(linear_control,angular_control,motorDrive_L,motorDrive_R);
    pid_drive(current_linear_x*2.0,current_angular_z*2.0,motorDrive_L,motorDrive_R);
    pid_plateform(anglePlatformY,hall_effect);
    // pwm_motor(current_linear_x,current_angular_z);
    // pwm_platform(platform_control,hall_effect);
    pwm_arm(linear_arm);
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
  msg_sub_motor.data.capacity = 8;

  msg_sub_sensors.data.data = sensors_data;
  msg_sub_sensors.data.capacity = 11;

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
  msg_pub_balance.data.capacity = 3;
  msg_pub_balance.data.size = 3;
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
  RCCHECK(rclc_subscription_init_default(&sub_pid, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "pid_parameters"));
  RCCHECK(rclc_subscription_init_default(&sub_arm_vel, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "arm_vel"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // --- 3. เริ่มต้น Timer และ Executor ---
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));

  // Executor รองรับ 2 งาน: 1 Subscription + 1 Timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor, &msg_sub_motor, &motor_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_arm_vel, &msg_sub_arm_vel, &arm_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_sensors, &msg_sub_sensors, &sensors_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_pid, &msg_sub_pid, &pid_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  // สั่งให้ระบบทำงาน (Spin)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}