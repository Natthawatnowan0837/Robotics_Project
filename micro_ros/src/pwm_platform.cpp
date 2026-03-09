// #include "main.h"

// void Platform_drive(int pinA, int pinB, float speed) {
//   speed = constrain(speed, -50, 50);
//   if (speed > 0) {
//     analogWrite(pinA, speed);
//     analogWrite(pinB, 0);
//   } else if (speed < 0) {
//     analogWrite(pinA, 0);
//     analogWrite(pinB, -speed);
//   } else {
//     analogWrite(pinA, 0);
//     analogWrite(pinB, 0);
//   }
// }

// void pwm_platform(float platform_control, float hall_effect) {
//     // 1. คำนวณค่า PWM เป้าหมายจากจอย (แปลง 0.0-1.0 เป็น 0-255)
//     float target_pwm = platform_control * 255.0f;

//     // 2. ตรวจสอบเงื่อนไข Hall Effect (Safety Limit)
//     // ถ้าเจอแม่เหล็ก (1.0) และจอยสั่งให้ถอยหลัง (ค่าติดลบ)
//     if (hall_effect > 0.5f && target_pwm < 0) {
//         target_pwm = 0; // บังคับเป็น 0 ทันที ไม่ให้ถอยหลังชน
//     }

//     // 3. ตรวจสอบ Deadzone (หลังจากเช็ค Limit แล้ว)
//     // ถ้าค่าจอยน้อยมาก หรือโดน Limit บังคับเป็น 0 ให้หยุดมอเตอร์
//     if (abs(target_pwm) < 12.0f) { // 12/255 ประมาณ 0.05 (Deadzone)
//         Platform_drive(PlatformLeft_R, PlatformLeft_L, 0);
//         Platform_drive(PlatformRight_R, PlatformRight_L, 0);
//         return;
//     }

//     // 4. สั่งงานมอเตอร์
//     // ฝั่งซ้ายคูณ -1.0 ตาม Logic การกลับทิศของคุณ
//     Platform_drive(PlatformLeft_R, PlatformLeft_L, target_pwm*-1);
//     Platform_drive(PlatformRight_R, PlatformRight_L, target_pwm*-1);

//     msg_pub_statePlatform.data.data[0] = target_pwm;
//     msg_pub_statePlatform.data.data[1] = hall_effect;

//     RCSOFTCHECK(rcl_publish(&pub_statePlatform, &msg_pub_statePlatform, NULL));
// }