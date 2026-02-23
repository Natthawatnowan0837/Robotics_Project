// #include "main.h"

// void pid_control(float r,float l){

    
// }
#include "main.h"
#include <PID_v1.h>

double L_Wheel_Setpoint, L_Wheel_Input, L_Wheel_Output;
double R_Wheel_Setpoint, R_Wheel_Input, R_Wheel_Output;

double wheel_Kp=40, wheel_Ki=0 , wheel_Kd=0;

PID L_wheel_PID(&L_Wheel_Input, &L_Wheel_Output, &L_Wheel_Setpoint, wheel_Kp, wheel_Ki, wheel_Kd, DIRECT);
PID R_wheel_PID(&R_Wheel_Input, &R_Wheel_Output, &R_Wheel_Setpoint, wheel_Kp, wheel_Ki, wheel_Kd, DIRECT);

void Motor_drive(int motor_R, int motor_L, float motorspeed) {
  motorspeed = constrain(motorspeed, -255, 255);
  if (motorspeed > 0) {
    analogWrite(motor_R, motorspeed);
    analogWrite(motor_L, 0);
  } else if (motorspeed < 0) {
    analogWrite(motor_R, 0);
    analogWrite(motor_L, -motorspeed);
  } else {
    analogWrite(motor_R, 0);
    analogWrite(motor_L, 0);
  }
}

void Robot_move(float linear_velocity, float angular_velocity, float enc_left, float enc_right) {
  // ... ส่วนคำนวณ PID เดิมของคุณ ...
  L_Wheel_Input = enc_left;
  R_Wheel_Input = enc_right;

  float wheel_base = 0.7;
  L_Wheel_Setpoint = linear_velocity - (angular_velocity * wheel_base / 2.0);
  R_Wheel_Setpoint = linear_velocity + (angular_velocity * wheel_base / 2.0);

  L_wheel_PID.Compute();
  R_wheel_PID.Compute();

  // ขับมอเตอร์
  Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, L_Wheel_Output);
  Motor_drive(WheelmotorRight_R, WheelmotorRight_L, R_Wheel_Output);

  // --- วิธีแก้: กำหนดค่าลงใน Array ทีละช่อง ---
  msg_vel_out.data.data[0] = (float)L_Wheel_Output;
  msg_vel_out.data.data[1] = (float)R_Wheel_Output;

  msg_setpoint.data.data[0] = (float)L_Wheel_Setpoint;
  msg_setpoint.data.data[1] = (float)R_Wheel_Setpoint;

  // สั่ง Publish
  RCSOFTCHECK(rcl_publish(&pub_vel_out, &msg_vel_out, NULL));
  RCSOFTCHECK(rcl_publish(&pub_setpoint, &msg_setpoint, NULL));
}

void init_PID() {
  L_wheel_PID.SetMode(AUTOMATIC);
  R_wheel_PID.SetMode(AUTOMATIC);
  
  // จำกัดช่วงของ Output (เช่น PWM ของ ESP32 คือ 0-255 หรือ 0-1023)
  L_wheel_PID.SetOutputLimits(-255, 255);
  R_wheel_PID.SetOutputLimits(-255, 255);
} 