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

void pwm_platform(float linear_x, bool hall_effect ,float gyro_body , float gyro_platform) {
    // 1. คำนวณ PWM เบื้องต้น (จาก 0.0 - 1.0 เป็น 0 - 255)
    float target_pwm = linear_x * 255.0f;

    // 2. ตรวจสอบเงื่อนไข Hall Effect (Limit Switch Logic)
    // ถ้า hall_effect เป็น true และกำลังสั่งให้ค่าติดลบ (ถอยลง/ถอยหลัง)
    if (hall_effect == true && target_pwm < 0) {
        target_pwm = 0; // บังคับให้เป็น 0 เพื่อหยุดการเคลื่อนที่ในทิศทางนั้น
    }

    // 3. ตรวจสอบ Deadzone (หลังจากเช็ค Limit แล้ว)
    if (abs(target_pwm) < 13.0f) { // 13/255 ประมาณ 0.05f
        Motor_drive_platform(PlatforLeft_R, PlatforLeft_L, 0);
        Motor_drive_platform(PlatforRight_R, PlatforRight_L, 0);
        return;
    }

    // 4. สั่งงานมอเตอร์ทั้ง 2 ตัว
    // ทิศทางของ Platform: ตัวหนึ่งหมุนปกติ อีกตัวหมุนย้อน (ตามกลไกของคุณ)
    Motor_drive_platform(PlatforLeft_R, PlatforLeft_L, target_pwm);
    Motor_drive_platform(PlatforRight_R, PlatforRight_L, target_pwm * -1.0f);

    // 5. ส่งค่ากลับไป ROS เพื่อตรวจสอบ (Debug)
    msg_platform_vel_out.data.data[0] = target_pwm; 
    msg_platform_vel_out.data.data[1] = target_pwm * -1.0f; 
    RCSOFTCHECK(rcl_publish(&pub_platform_vel_out, &msg_platform_vel_out, NULL));
}