#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h> 
#include <rcutils/logging_macros.h>

// --- Configuration Pins ---
#define WheelmotorLeft_R 19
#define WheelmotorLeft_L 18
#define WheelmotorRight_R 5
#define WheelmotorRight_L 17

// --- Macros ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- แชร์ฟังก์ชันและตัวแปร (Extern) ---
void error_loop();
extern rcl_publisher_t pub_vel_out,pub_setpoint;
extern std_msgs__msg__Float32MultiArray msg_vel_out,msg_setpoint;
extern unsigned long lastMsgTime; // เพิ่มเพื่อให้ไฟล์อื่นมองเห็นตัวแปรเวลา


// --- ฟังก์ชันต้นแบบ (Prototypes) ---
// แก้ไข: เปลี่ยนชื่อพารามิเตอร์ไม่ให้ซ้ำกัน (l กับ r)
// void init_PID();
// void pid_motor(float linear , float angular, float enc_left, float enc_right);
void pwm(float linear, float angular);
#endif