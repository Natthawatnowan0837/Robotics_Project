#include "main.h"

void Arm_drive_platform(int pinA, int pinB, float speed) {
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

void pwm_platform(float linear_x) {
    // 1. ตรวจสอบ Deadzone เพื่อหxยุดมอเตอร์ (ป้องกันมอเตอร์ครางแต่ไม่หมุน)
    if (abs(linear_x) < 0.05f) {
        Arm_drive_platform(PlatformLeft_R, PlatformLeft_L, 0);
        Arm_drive_platform(PlatformRight_R, PlatformRight_L, 0);
        return;
    }

    // 2. คำนวณ PWM (จาก 0.0 - 1.0 เป็น 0 - 255)
    float target_pwm = linear_x * 255.0f;

    // 3. สั่งงานมอเตอร์ทั้ง 2 ตัว
    // หมายเหตุ: หากมอเตอร์ตัวใดตัวหนึ่งหมุนสลับทาง ให้คูณด้วย -1.0f ที่ตัวนั้น
    Arm_drive_platform(PlatformLeft_R, PlatformLeft_L, target_pwm);
    Arm_drive_platform(PlatformRight_R, PlatformRight_L, target_pwm* -1.0f);

    msg_platform_vel_out.data.data[0] = target_pwm; // สำหรับ Debug
    msg_platform_vel_out.data.data[1] = target_pwm; // สำหรับ Debug
    RCSOFTCHECK(rcl_publish(&pub_platform_vel_out, &msg_platform_vel_out, NULL));
}