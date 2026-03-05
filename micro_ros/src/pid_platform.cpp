#include "main.h"
// ตัวแปรสำหรับ Platform PID
float Platform_Setpoint = 0.0; // เป้าหมายคือ 0 องศา
float Platform_Input;          // ค่าจาก Gyro
float Platform_Output;         // ค่า PWM ที่จะส่งไปมอเตอร์

// ค่า Gain สำหรับ Platform (ต้อง Tuning ใหม่)
float P_Kp = 15.0, P_Ki = 0.0, P_Kd = 0.0; 

QuickPID Platform_PID(&Platform_Input, &Platform_Output, &Platform_Setpoint);


void Platform_drive(int pinA, int pinB, float speed) {
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

void init_Platform_PID() {
    Platform_PID.SetTunings(P_Kp, P_Ki, P_Kd);
    Platform_PID.SetOutputLimits(-255, 255); // Output เป็น PWM
    Platform_PID.SetMode(Platform_PID.Control::automatic);
}

void pid_platform(bool hall_effect, float gyro_platform) {
    // 1. อัปเดต Input จาก Gyro ที่ผ่าน Filter มาแล้ว
    Platform_Input = gyro_platform;
    Platform_PID.Compute();
    float target_pwm = Platform_Output;

    if (hall_effect && target_pwm < 0) {
        target_pwm = 0; // หยุดถ้าชน Limit
    }

    // 4. ตรวจสอบ Deadzone เพื่อไม่ให้มอเตอร์คราง (Humming) เมื่อใกล้ 0
    if (abs(target_pwm) < 15.0f) {
        target_pwm = 0;
    }

    // 5. สั่งงานมอเตอร์ (ใช้ฟังก์ชันเดิมที่คุณเขียนไว้)
    // หมายเหตุ: เช็คทิศทาง (+/-) ให้ตรงกับหน้างานจริง
    Platform_drive(PlatforLeft_R, PlatforLeft_L, target_pwm);
    Platform_drive(PlatforRight_R, PlatforRight_L, target_pwm * -1.0f);

    // 6. ส่งค่า Debug กลับไป ROS
    msg_platform_vel_out.data.data[0] = target_pwm;
    msg_platform_vel_out.data.data[1] = target_pwm * -1.0f;
    rcl_publish(&pub_platform_vel_out, &msg_platform_vel_out, NULL);
}