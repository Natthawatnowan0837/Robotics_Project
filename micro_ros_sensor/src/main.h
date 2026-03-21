#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <MS5611.h>
#include <micro_ros_arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/vector3.h> 
#include <std_msgs/msg/bool.h>
#include <math.h>
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
extern MS5611 ms5611;

// extern pressure;

extern rcl_timer_t timer; // เพิ่ม timer
extern float filtered_rps_l, filtered_rps_r;
extern float angleBodyX, angleBodyY, angleBodyZ;
extern float anglePlatformX, anglePlatformY, anglePlatformZ;
extern bool encoder_online[2];

extern rcl_publisher_t pub_motor,pub_sensors;
            
extern std_msgs__msg__Float32MultiArray msg_motor,msg_sensors; 

extern const int hallPin ;     // ขาที่ต่อกับ Out ของเซนเซอร์
extern int hallState ;  
// 
// --- Kalman Filter Class (ไว้ที่นี่ที่เดียว) ---
class SimpleKalmanFilter {
  private:
    float err_measure, err_estimate, q, current_estimate, last_estimate, kalman_gain;
    bool is_initialized = false;
  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q_val) {
      err_measure = mea_e; err_estimate = est_e; q = q_val;
    }
    float updateEstimate(float mea) {
      if (!is_initialized) { last_estimate = mea; current_estimate = mea; is_initialized = true; return current_estimate; }
      kalman_gain = err_estimate / (err_estimate + err_measure);
      current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
      err_estimate = (1.0f - kalman_gain) * err_estimate + fabsf(last_estimate - current_estimate) * q;
      last_estimate = current_estimate;
      return current_estimate;
    }
};

float lowpassFilter(float input, float prev_output, float alpha);
float medianFilter(float* data, int size);

void Encoder_motor();
void Encoder_arm();
void Gyro();
void hall_effect();
void pressure();
void tcaSelect(uint8_t i);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void error_loop();
void calibrateSensors();
#endif