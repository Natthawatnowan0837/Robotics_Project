#include "main.h"

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

void pwm_motor(float linear, float angular) {
  // 1. ตรวจสอบ Deadzone (ถ้าค่าน้อยมากให้หยุดมอเตอร์ทันที)
  if (abs(linear) < 0.05f && abs(angular) < 0.05f) {
    Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, 0);
    Motor_drive(WheelmotorRight_R, WheelmotorRight_L, 0);

    msg_vel_out.data.data[0] = 0.0f;
    msg_vel_out.data.data[1] = 0.0f;
    RCSOFTCHECK(rcl_publish(&pub_vel_out, &msg_vel_out, NULL));
    return;
  }

  // 2. คำนวณค่า PWM พื้นฐาน (Differential Drive Logic แบบง่าย)
  // linear: เดินหน้า/ถอยหลัง (-1.0 ถึง 1.0)
  // angular: เลี้ยวซ้าย/ขวา (-1.0 ถึง 1.0)
  float raw_l = linear - angular;
  float raw_r = linear + angular;

  // 3. Scale ค่าจากช่วง 1.0 ไปเป็น 255.0 และ Constrain ไม่ให้เกินช่วง PWM
  float pwm_l = constrain(raw_l * 255.0f, -255.0f, 255.0f);
  float pwm_r = constrain(raw_r * 255.0f, -255.0f, 255.0f);

  // 4. สั่งงานมอเตอร์ (คูณ -1.0 ตามทิศทาง Hardware ของคุณ)
  Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, pwm_l * -1.0f);
  Motor_drive(WheelmotorRight_R, WheelmotorRight_L, pwm_r * -1.0f);

  // 5. ส่งค่ากลับไป ROS เพื่อตรวจสอบ
  msg_vel_out.data.data[0] = pwm_l;
  msg_vel_out.data.data[1] = pwm_r;
  RCSOFTCHECK(rcl_publish(&pub_vel_out, &msg_vel_out, NULL));
}

