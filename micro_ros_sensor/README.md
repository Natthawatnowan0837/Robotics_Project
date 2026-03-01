ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

ls /dev/ttyUSB* 

export ROS_DOMAIN_ID=0
ros2 daemon stop
ros2 daemon start
ros2 topic list

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <MPU6050_light.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/vector3.h>

// --- Configuration ---
#define TCA_ADDR 0x70
#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 2
#define MEDIAN_SIZE 5

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- Objects ---
AS5600 as5600(&Wire);
MPU6050 mpuBody(Wire);      // IMU ตัวที่ 1
MPU6050 mpuPlatform(Wire);  // IMU ตัวที่ 2

// --- State Variables ---
bool imu_online[2] = {false, false};       
bool encoder_online[4] = {false, false, false, false}; 
unsigned long last_time_motor = 0;
float last_angle_ml = 0; 
float last_angle_mr = 0; 

float buffer_l[MEDIAN_SIZE];
float buffer_r[MEDIAN_SIZE];
float filtered_rps_l = 0;
float filtered_rps_r = 0;

// --- Micro-ROS Objects ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_publisher_t pub_motor_rps, pub_arm_degrees;
rcl_publisher_t pub_imu_body, pub_imu_plateform;

std_msgs__msg__Float32MultiArray msg_motor_rps; 
std_msgs__msg__Float32MultiArray msg_arm_degrees; 
geometry_msgs__msg__Vector3 msg_imu_body, msg_imu_plateform;

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
  delayMicroseconds(10); // รอให้สัญญาณ I2C นิ่งหลังสลับ Channel
}

bool checkI2CAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

float getMedian(float *data, int n) {
    float temp[n];
    memcpy(temp, data, n * sizeof(float));
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (temp[i] > temp[j]) {
                float swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }
    return temp[n / 2];
}

// --- Sensor Reading Functions ---

void read_IMU() {
  // Body IMU (Channel 0)
  if (imu_online[0]) {
    tcaSelect(0);
    mpuBody.update(); 
    msg_imu_body.x = mpuBody.getAngleX(); 
    msg_imu_body.y = mpuBody.getAngleY(); 
    msg_imu_body.z = mpuBody.getAngleZ();
    RCSOFTCHECK(rcl_publish(&pub_imu_body, &msg_imu_body, NULL));
  }

  // Platform IMU (Channel 1)
  if (imu_online[1]) {
    tcaSelect(1);
    mpuPlatform.update();
    msg_imu_plateform.x = mpuPlatform.getAngleX(); 
    msg_imu_plateform.y = mpuPlatform.getAngleY(); 
    msg_imu_plateform.z = mpuPlatform.getAngleZ();
    RCSOFTCHECK(rcl_publish(&pub_imu_plateform, &msg_imu_plateform, NULL));
  }
}

void Encoder_motor() {
  unsigned long current_time = micros();
  float dt = (float)(current_time - last_time_motor) / 1000000.0;
  if (dt < 0.005) return; 

  const float RAW_TO_DEG = (360.0 / 4096.0) * -1.0;
  const float alpha = 0.2; 

  // Motor Left (Channel 2)
  if (encoder_online[0]) {
    tcaSelect(2);
    float current_angle = as5600.readAngle() * RAW_TO_DEG;
    float delta_angle = current_angle - last_angle_ml;
    if (delta_angle > 180) delta_angle -= 360;
    else if (delta_angle < -180) delta_angle += 360;

    float raw_rps = (delta_angle / 360.0) / dt;
    for (int i = MEDIAN_SIZE - 1; i > 0; i--) buffer_l[i] = buffer_l[i - 1];
    buffer_l[0] = raw_rps;
    filtered_rps_l = (alpha * getMedian(buffer_l, MEDIAN_SIZE)) + ((1.0 - alpha) * filtered_rps_l);
    
    msg_motor_rps.data.data[0] = filtered_rps_l;
    last_angle_ml = current_angle;
  }

  // Motor Right (Channel 3)
  if (encoder_online[1]) {
    tcaSelect(3);
    float current_angle = as5600.readAngle() * RAW_TO_DEG;
    float delta_angle = current_angle - last_angle_mr;
    if (delta_angle > 180) delta_angle -= 360;
    else if (delta_angle < -180) delta_angle += 360;

    float raw_rps = (delta_angle / 360.0) / dt;
    for (int i = MEDIAN_SIZE - 1; i > 0; i--) buffer_r[i] = buffer_r[i - 1];
    buffer_r[0] = raw_rps;
    filtered_rps_r = (alpha * getMedian(buffer_r, MEDIAN_SIZE)) + ((1.0 - alpha) * filtered_rps_r);

    msg_motor_rps.data.data[1] = filtered_rps_r;
    last_angle_mr = current_angle;
  }

  msg_motor_rps.data.size = 2;
  RCSOFTCHECK(rcl_publish(&pub_motor_rps, &msg_motor_rps, NULL));
  last_time_motor = current_time;
}

void Encoder_arm() {
  const float RAW_TO_DEG = 360.0 / 4096.0;
  
  if (encoder_online[2]) { tcaSelect(4); msg_arm_degrees.data.data[0] = as5600.readAngle() * RAW_TO_DEG; }
  if (encoder_online[3]) { tcaSelect(5); msg_arm_degrees.data.data[1] = as5600.readAngle() * RAW_TO_DEG; }

  msg_arm_degrees.data.size = 2;
  RCSOFTCHECK(rcl_publish(&pub_arm_degrees, &msg_arm_degrees, NULL));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer != NULL) {
    read_IMU();      
    Encoder_motor(); 
    Encoder_arm();   
  }
}

// --- Main Setup ---

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); 
  
  set_microros_transports();
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // ใช้ 100kHz เพื่อความเสถียรสูงสุดผ่าน Multiplexer

  // Init IMUs
  tcaSelect(0);
  if (checkI2CAddress(0x68)) { 
    if (mpuBody.begin() == 0) {
      mpuBody.calcOffsets(); 
      imu_online[0] = true;
    }
  }

  tcaSelect(1);
  if (checkI2CAddress(0x68)) { 
    if (mpuPlatform.begin() == 0) {
      mpuPlatform.calcOffsets(); 
      imu_online[1] = true;
    }
  }
  
  // Init Encoders
  for (uint8_t i = 2; i <= 5; i++) {
    tcaSelect(i);
    if (checkI2CAddress(0x36)) { 
      as5600.begin(); 
      encoder_online[i-2] = true; 
    }
  }

  // Memory Allocation
  static float rps_data[2];
  msg_motor_rps.data.capacity = 2;
  msg_motor_rps.data.data = rps_data;

  static float arm_data[2];
  msg_arm_degrees.data.capacity = 2;
  msg_arm_degrees.data.data = arm_data;

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_sensor_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_motor_rps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motor_rps_array"));
  RCCHECK(rclc_publisher_init_default(&pub_arm_degrees, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "arm_deg_array"));
  RCCHECK(rclc_publisher_init_default(&pub_imu_body, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "imu_body"));
  RCCHECK(rclc_publisher_init_default(&pub_imu_plateform, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "imu_plateform"));

  // Timer 20ms (50Hz)
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  digitalWrite(LED_PIN, HIGH); 
  last_time_motor = micros();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}