#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <MPU6050_light.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>
#include <rcutils/logging_macros.h>


#include "main.h"
// --- Configuration ---
#define TCA_ADDR 0x70
#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 2

AS5600 as5600(&Wire);
MPU6050 mpu(Wire);

// --- State Variables ---
bool imu_online[2] = {false, false};       
bool encoder_online[4] = {false, false, false, false}; 
unsigned long last_time_motor = 0;
unsigned long lastMsgTime = 0;
float last_angle_ml = 0; 
float last_angle_mr = 0; 
float raw_rps_l = 0.0; // ประกาศไว้ตรงนี้เพื่อให้ทุกฟังก์ชันมองเห็น
float raw_rps_r = 0.0;

float lpf_alpha = 0.2;
float enc_left = 0.0;
float enc_right = 0.0;

float window_l[3] = {0, 0, 0};
float window_r[3] = {0, 0, 0};

// ฟังก์ชันจิ๋วสำหรับหาค่ากลางของเลข 3 ตัว
float get_median3(float new_val, float* window) {
    window[2] = window[1];
    window[1] = window[0];
    window[0] = new_val;

    float a = window[0];
    float b = window[1];
    float c = window[2];

    // คืนค่ากลาง (Median)
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

// --- Publishers ---
rcl_publisher_t pub_encoder_motorLeft, pub_encoder_motorRight, pub_encoder_armLeft, pub_encoder_armRight;
rcl_publisher_t pub_motor_rps; // ยุบรวมเหลือตัวเดียว

rcl_publisher_t pub_imu_body, pub_imu_plateform;
rcl_publisher_t pub_vel_out ,pub_setpoint;

// ตัวเก็บ Message สำหรับตอนจะส่ง


// --- Subscribers ---
rcl_subscription_t sub_cmd_vel;

// --- Messages Storage ---
std_msgs__msg__Float32 msg_encoder_motorLeft, msg_encoder_motorRight, msg_encoder_armLeft, msg_encoder_armRight;
std_msgs__msg__Float32MultiArray msg_motor_rps , msg_vel_out,msg_setpoint; // ตัวเก็บข้อมูล Array
geometry_msgs__msg__Vector3 msg_imu_body, msg_imu_plateform ;
geometry_msgs__msg__Twist msg_move_cmd;

static float vel_out_buffer[2];
static float setpoint_buffer[2];
// --- Helper Functions ---

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

bool checkI2CAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// --- Callback Functions ---
void vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_vel = msg->linear.x;
  float angular_vel = msg->angular.z;

  // อัปเดตค่าความเร็วจาก Encoder ที่ได้จากฟังก์ชัน Encoder_motor()
  enc_left = raw_rps_l; 
  enc_right = raw_rps_r;

  // เรียกใช้ Robot_move เพื่อคำนวณ IK และ PID
  Robot_move(linear_vel, angular_vel, enc_left, enc_right);
  
  lastMsgTime = millis();
}
// --- Sensor Reading Functions ---

void read_IMU() {
  if (imu_online[0]) {
    tcaSelect(0); mpu.update();
    msg_imu_body.x = mpu.getAngleX(); msg_imu_body.y = mpu.getAngleY(); msg_imu_body.z = mpu.getAngleZ();
    RCSOFTCHECK(rcl_publish(&pub_imu_body, &msg_imu_body, NULL));
  }
  if (imu_online[1]) {
    tcaSelect(1); mpu.update();
    msg_imu_plateform.x = mpu.getAngleX(); msg_imu_plateform.y = mpu.getAngleY(); msg_imu_plateform.z = mpu.getAngleZ();
    RCSOFTCHECK(rcl_publish(&pub_imu_plateform, &msg_imu_plateform, NULL));
  }
}
void init_motor_rps_msg() {
    static float data_buffer[2]; // สร้างถังพักข้อมูลขนาด 2 ช่อง
    msg_motor_rps.data.capacity = 2;
    msg_motor_rps.data.size = 2;
    msg_motor_rps.data.data = data_buffer;
}

void init_debug_msgs() {
    // เตรียม Memory สำหรับ Velocity Output
    msg_vel_out.data.capacity = 2;
    msg_vel_out.data.size = 2;
    msg_vel_out.data.data = vel_out_buffer;

    // เตรียม Memory สำหรับ Setpoint
    msg_setpoint.data.capacity = 2;
    msg_setpoint.data.size = 2;
    msg_setpoint.data.data = setpoint_buffer;
}

void Encoder_motor() {
  unsigned long current_time = micros();
  float dt = (float)(current_time - last_time_motor) / 1000000.0;
  if (dt < 0.01) return;

  const float RAW_TO_DEG = 360.0 / 4096.0;

  // --- Motor Left (Index 0) ---
  if (encoder_online[0]) {
    tcaSelect(2);
    float current_angle = as5600.readAngle() * RAW_TO_DEG;
    float delta_angle = current_angle - last_angle_ml;
    if (delta_angle > 180) delta_angle -= 360;
    else if (delta_angle < -180) delta_angle += 360;

    float raw_rps_l_new = (delta_angle / 360.0) / dt;

    // 1. ผ่าน Median Filter (กำจัด Spike)
    float median_l = get_median3(raw_rps_l_new, window_l);

    // 2. ผ่าน Low Pass Filter (เกลี่ยให้เนียน)
    raw_rps_l = (lpf_alpha * median_l) + (1.0 - lpf_alpha) * raw_rps_l;
    
    if (abs(raw_rps_l) < 0.01) raw_rps_l = 0.0;
    msg_motor_rps.data.data[0] = raw_rps_l;
    last_angle_ml = current_angle;
  }

  // --- Motor Right (Index 1) ---
  if (encoder_online[1]) {
    tcaSelect(3);
    float current_angle = as5600.readAngle() * RAW_TO_DEG;
    float delta_angle = current_angle - last_angle_mr;
    if (delta_angle > 180) delta_angle -= 360;
    else if (delta_angle < -180) delta_angle += 360;

    float raw_rps_r_new = (delta_angle / 360.0) / dt;

    // 1. ผ่าน Median Filter
    float median_r = get_median3(raw_rps_r_new, window_r);

    // 2. ผ่าน Low Pass Filter
    raw_rps_r = (lpf_alpha * median_r) + (1.0 - lpf_alpha) * raw_rps_r;
    
    if (abs(raw_rps_r) < 0.01) raw_rps_r = 0.0;
    msg_motor_rps.data.data[1] = raw_rps_r;
    last_angle_mr = current_angle;
  }

  RCSOFTCHECK(rcl_publish(&pub_motor_rps, &msg_motor_rps, NULL));
  last_time_motor = current_time;
}

void Encoder_arm() {
  const float RAW_TO_DEG = 360.0 / 4096.0;
  // Arm Left (CH 4)
  if (encoder_online[2]) {
    tcaSelect(4); msg_encoder_armLeft.data = as5600.readAngle() * RAW_TO_DEG;
    RCSOFTCHECK(rcl_publish(&pub_encoder_armLeft, &msg_encoder_armLeft, NULL));
  }
  // Arm Right (CH 5)
  if (encoder_online[3]) {
    tcaSelect(5); msg_encoder_armRight.data = as5600.readAngle() * RAW_TO_DEG;
    RCSOFTCHECK(rcl_publish(&pub_encoder_armRight, &msg_encoder_armRight, NULL));
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    read_IMU();      
    Encoder_motor(); 
    Encoder_arm();   
  }
}

// --- Main Setup ---

void setup() {
  init_PID();
  pinMode(LED_PIN, OUTPUT);
  set_microros_transports();
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Scan Hardware: IMU
  for (uint8_t i = 0; i <= 1; i++) {
    tcaSelect(i);
    if (checkI2CAddress(0x68)) { 
      if (mpu.begin() == 0) imu_online[i] = true; 
    }
  }
  
  // Scan Hardware: Encoders
  for (uint8_t i = 2; i <= 5; i++) {
    tcaSelect(i);
    if (checkI2CAddress(0x36)) { 
      as5600.begin(); 
      encoder_online[i-2] = true; 
    }
  }

  // เตรียมหน่วยความจำสำหรับ MultiArray
  init_motor_rps_msg();
  init_debug_msgs();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_sensor_node", "", &support));

  // --- Init Publishers ---
  
  // 1. ยุบรวมล้อซ้าย-ขวาเป็น Topic เดียว (motor_rps_array)
  RCCHECK(rclc_publisher_init_default(&pub_motor_rps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motor_rps_array"));

  // 2. Publisher อื่นๆ (เหมือนเดิม)
  RCCHECK(rclc_publisher_init_default(&pub_encoder_armLeft, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "arm_left_deg"));
  RCCHECK(rclc_publisher_init_default(&pub_encoder_armRight, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "arm_right_deg"));  
  RCCHECK(rclc_publisher_init_default(&pub_imu_body, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "imu_body"));
  RCCHECK(rclc_publisher_init_default(&pub_imu_plateform, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "imu_plateform"));
  RCCHECK(rclc_publisher_init_default(&pub_vel_out, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "velocity_feedback"));
  RCCHECK(rclc_publisher_init_default(&pub_setpoint, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "setpoint"));

  // --- Init Subscriber ---
  RCCHECK(rclc_subscription_init_default(
    &sub_cmd_vel, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), 
    "cmd_vel"
  ));

  // --- Init Timer ---
  // แนะนำปรับเป็น 20ms เพื่อให้ PID ทำงานได้เนียนขึ้น
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));
  
  // --- Init Executor ---
  // Handles = 1 Timer + 1 Subscription = 2 Handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_move_cmd, &vel_callback, ON_NEW_DATA));
  
  digitalWrite(LED_PIN, HIGH); 
  last_time_motor = millis();
  lastMsgTime = millis();
}
void loop() {

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}