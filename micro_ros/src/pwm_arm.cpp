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

void pwm_arm(float linear_x) {
    // 1. คำนวณค่า PWM (Scale จาก 1.0 เป็น 255.0)
    // ถ้า linear_x เป็น 1.0 (RT) -> pwm = 255 (ยืดแขน/ขึ้น)
    // ถ้า linear_x เป็น -1.0 (LT) -> pwm = -255 (หดแขน/ลง)
    float target_pwm = linear_x * 255.0f;
    target_pwm = constrain(target_pwm, -255.0f, 255.0f);

    // 2. ตรวจสอบ Deadzone เล็กน้อย
    if (abs(target_pwm) < 10.0f) {
        target_pwm = 0;
    }

    // 3. สั่งงานมอเตอร์แขน (ใช้ขา Pin ที่คุณกำหนดไว้สำหรับ Arm)
    // หมายเหตุ: เปลี่ยน Arm_Motor_R/L เป็นชื่อตัวแปร Pin ที่คุณตั้งไว้ใน main.h
    Motor_drive(ArmLeft_R, ArmLeft_L, target_pwm);
    Motor_drive(ArmRight_R, ArmRight_L, target_pwm * -1.0f); // สมมติว่าตัวนี้ต้องหมุนย้อนกัน

    // 4. ส่งค่ากลับไป ROS เพื่อ Debug (ใช้ Publisher ของ Arm โดยเฉพาะ)
    msg_arm_vel_out.data.data[0] = target_pwm; 
    RCSOFTCHECK(rcl_publish(&pub_arm_vel_out, &msg_arm_vel_out, NULL));
}

