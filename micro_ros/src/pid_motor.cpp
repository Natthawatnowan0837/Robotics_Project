#include <QuickPID.h>
#include "main.h"

float L_Wheel_vel, L_Wheel_Setpoint, L_Wheel_Input, L_Wheel_Output;
float R_Wheel_vel, R_Wheel_Setpoint, R_Wheel_Input, R_Wheel_Output;

float L_Kp = 13.0, L_Ki = 0.0, L_Kd = 0.0;
float R_Kp = 13.2, R_Ki = 0.0, R_Kd = 0.0;

QuickPID L_wheel_PID(&L_Wheel_Input, &L_Wheel_Output, &L_Wheel_Setpoint);
QuickPID R_wheel_PID(&R_Wheel_Input, &R_Wheel_Output, &R_Wheel_Setpoint);

void Motor_drive(int pinA, int pinB, float speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(pinA, speed);
    analogWrite(pinB, 0);
  } else if (speed < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -speed);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

void init_PID() {
  L_wheel_PID.SetTunings(L_Kp, L_Ki, L_Kd);
  R_wheel_PID.SetTunings(R_Kp, R_Ki, R_Kd);

  L_wheel_PID.SetOutputLimits(-255, 255);
  R_wheel_PID.SetOutputLimits(-255, 255);

  L_wheel_PID.SetMode(L_wheel_PID.Control::automatic);
  R_wheel_PID.SetMode(R_wheel_PID.Control::automatic);
}

void pid_motor(float linear, float angular, float enc_left, float enc_right) {
  float wheel_base = 0.7;      
  float wheel_diameter = 0.15; 
  float circumference = wheel_diameter * PI;

  L_Wheel_vel = linear - (angular * wheel_base / 2.0);
  R_Wheel_vel = linear + (angular * wheel_base / 2.0);

  L_Wheel_Setpoint = L_Wheel_vel / circumference;
  R_Wheel_Setpoint = R_Wheel_vel / circumference;

  L_Wheel_Input = enc_left; 
  R_Wheel_Input = enc_right; 

  L_wheel_PID.Compute();
  R_wheel_PID.Compute();

  Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, L_Wheel_Output * -1.0);
  Motor_drive(WheelmotorRight_R, WheelmotorRight_L, R_Wheel_Output * -1.0);

  msg_vel_out.data.data[0] = (float)L_Wheel_Output;
  msg_vel_out.data.data[1] = (float)R_Wheel_Output;

  msg_setpoint.data.data[0] = (float)L_Wheel_Setpoint;
  msg_setpoint.data.data[1] = (float)R_Wheel_Setpoint;

  RCSOFTCHECK(rcl_publish(&pub_vel_out, &msg_vel_out, NULL));
  RCSOFTCHECK(rcl_publish(&pub_setpoint, &msg_setpoint, NULL));
}
