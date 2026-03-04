#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <micro_ros_arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/vector3.h> 
#include <std_msgs/msg/bool.h>
#define TCA_ADDR 0x70
// Macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define MEDIAN_SIZE 5

// Extern Objects
extern AS5600 as5600_motor; 
extern AS5600 as5600_arm;
extern Adafruit_MPU6050 mpuBody;
extern Adafruit_MPU6050 mpuPlatform;
// extern pressure;

extern rcl_timer_t timer; // เพิ่ม timer
extern float filtered_rps_l, filtered_rps_r;
extern float angleBodyX, angleBodyY, angleBodyZ;
extern float anglePlatformX, anglePlatformY, anglePlatformZ;
extern bool encoder_online[2];

extern rcl_publisher_t pub_motor_rps, pub_arm_degrees,
                pub_imu_body, pub_imu_platform,
                pub_hall_effect;

extern std_msgs__msg__Bool msg_hall_effect;
extern geometry_msgs__msg__Vector3 msg_imu_body, msg_imu_platform;
extern std_msgs__msg__Float32MultiArray msg_motor_rps  ,msg_arm_degrees; 

extern const int hallPin ;     // ขาที่ต่อกับ Out ของเซนเซอร์
extern int hallState ;  
// Prototypes
float getMedian(float* data, int size);
void Encoder_motor();
void Encoder_arm();
void Gyro();
void hall_effect();
void tcaSelect(uint8_t i);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void error_loop();

#endif