#include "main.h"

void Motor_drive_platform(int pinA, int pinB, float speed) {
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

void pwm_platform(bool hall_effect_triggered, float current_linear) {
    // 1. ตรวจสอบ Deadzone เพื่อหยุดมอเตอร์
    if (abs(current_linear) < 0.05f) {
        Motor_drive_platform(PlatformLeft_R, PlatformLeft_L, 0);
        Motor_drive_platform(PlatformRight_R, PlatformRight_L, 0);
        return;
    }

    // 2. คำนวณ PWM (0.0 - 1.0 -> 0 - 255)
    float target_pwm = current_linear * 255.0f;

    // 3. ตรรกะป้องกัน (Safety Logic)
    // ถ้า hall_effect_triggered เป็น true และ target_pwm < 0 (พยายามถอยหลัง)
    // ให้บังคับค่าเป็น 0 เพื่อหยุดมอเตอร์ทันที
    if (hall_effect_triggered && target_pwm < 0) {
        target_pwm = 0;
    }

    // 4. สั่งงานมอเตอร์
    // ตัวขวาคูณ -1.0f ตาม Logic เดิมที่คุณเขียนไว้ (สันนิษฐานว่ามอเตอร์ติดตั้งกลับด้านกัน)
    Motor_drive_platform(PlatformLeft_R, PlatformLeft_L, target_pwm);
    Motor_drive_platform(PlatformRight_R, PlatformRight_L, target_pwm * -1.0f);

    // ส่งค่า Debug
    msg_platform_vel_out.data.data[0] = target_pwm;
    msg_platform_vel_out.data.data[1] = target_pwm;
    RCSOFTCHECK(rcl_publish(&pub_platform_vel_out, &msg_platform_vel_out, NULL));
}
