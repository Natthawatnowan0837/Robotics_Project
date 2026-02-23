// #include "main.h"
// // ระยะห่างระหว่างล้อ (เมตร)
// const float WHEEL_SEPARATION = 1.0; 
// // ฟังก์ชันสั่งงาน PWM แยกแต่ละมอเตอร์
// void set_motor_speed(int motor_F, int motor_R, float motorspeed) {
//   // Deadzone ป้องกันมอเตอร์สั่น
//   if (abs(motorspeed) < 0.05) {
//     analogWrite(motor_F, 0);
//     analogWrite(motor_R, 0);
//     return;
//   }

//   motorspeed = constrain(motorspeed, -1.0, 1.0);
//   int pwmValue = abs(motorspeed * 255);

//   if (motorspeed > 0) {
//     analogWrite(motor_F, pwmValue);
//     analogWrite(motor_R, 0);
//   } else if (motorspeed < 0) {
//     analogWrite(motor_F, 0);
//     analogWrite(motor_R, pwmValue);
//   } else {
//     analogWrite(motor_F, 0);
//     analogWrite(motor_R, 0);
//   }
// }

// // ฟังก์ชันหลักที่รับค่า Linear (v) และ Angular (w)
// void driveMotor(float v, float w) {
//   // --- 1. Differential Drive Kinematics ---
//   float speed_L = v - (w * WHEEL_SEPARATION / 2.0);
//   float speed_R = v + (w * WHEEL_SEPARATION / 2.0);

//   // --- 2. สั่งงานมอเตอร์จริง ---
//   set_motor_speed(WheelmotorLeft_R, WheelmotorLeft_L, L_Wheel_Output);
//   set_motor_speed(WheelmotorRight_R, WheelmotorRight_L, R_Wheel_Output);

//   // --- 3. Publish ค่า PWM ของล้อซ้ายกลับไปเช็ค (0-255) ---
//   // นำค่า speed_L มาผ่าน Logic เดียวกับใน set_motor_speed เพื่อดูค่า PWM จริง
//   float constrained_speed = constrain(speed_L, -1.0, 1.0);
//   int current_pwm = (abs(constrained_speed) < 0.05) ? 0 : abs(constrained_speed * 255);

//   // ส่งค่า PWM ออกไป (ถ้าถอยหลังค่าจะเป็นบวกตาม abs)
//   msg_vel_out.data = (float)current_pwm; 
  
//   RCSOFTCHECK(rcl_publish(&pub_vel_out, &msg_vel_out, NULL));
// }

