#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h> 
#include <rcutils/logging_macros.h>


#define WheelmotorLeft_R 19
#define WheelmotorLeft_L 18
#define WheelmotorRight_R 16
#define WheelmotorRight_L 17

#define PlatformLeft_R 13
#define PlatformLeft_L 12
#define PlatformRight_R  27 
#define PlatformRight_L 14

#define ArmLeft_R 26
#define ArmLeft_L 25
#define ArmRight_R 32
#define ArmRight_L 33

// --- Macros ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern float motorDrive_L; 
extern float motorDrive_R;
extern float motorArm_L;
extern float motorArm_R;
extern float linear_control; 
extern float angular_control;
extern float platform_control;
extern float arm_control;
extern float pid_driveL_parameters[3];
extern float pid_driveR_parameters[3];
extern float pid_platform_parameters[3];

extern rcl_publisher_t pub_drive,pub_statePlatform,pub_stateArm,pub_balance;

extern std_msgs__msg__Float32 msg_pub_stateArm;
extern std_msgs__msg__Float32MultiArray msg_pub_drive,msg_pub_statePlatform,msg_pub_balance; 

// // --- แชร์ฟังก์ชันและตัวแปร (Extern) ---
void init_PID();
void pid_drive(float linear, float angular, float motorDrive_L, float motorDrive_R);

void init_plateformPID();
void pid_plateform(float anglePlatformY,float hall_effect);

// void pwm_motor(float linear_control,float angular_control);
// void pwm_platform(float platform_control,float hall_effect);
// void pwm_arm(float arm_control);


#endif