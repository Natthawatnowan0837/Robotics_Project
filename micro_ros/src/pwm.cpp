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

void pwm(float linear, float angular) {
  // 1. ตรวจสอบเงื่อนไขหยุดรถก่อนเป็นอันดับแรก
  if (abs(linear) < 0.01f && abs(angular) < 0.01f) {
    Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, 0);
    Motor_drive(WheelmotorRight_R, WheelmotorRight_L, 0);

    // ล้างค่าข้อมูลสำหรับส่งกลับไป ROS
    msg_vel_out.data.data[0] = 0.0f;
    msg_vel_out.data.data[1] = 0.0f;
    msg_setpoint.data.data[0] = 0.0f;
    msg_setpoint.data.data[1] = 0.0f;
    
    RCSOFTCHECK(rcl_publish(&pub_vel_out, &msg_vel_out, NULL));
    return; // จบการทำงานทันที ไม่ต้องลงไปคำนวณด้านล่างต่อ
  }

  // 2. ตั้งค่าคงที่หุ่นยนต์
  const float wheel_base = 0.70f;
  const float wheel_diameter = 0.15f;
  const float circumference = wheel_diameter * PI;

  // 3. คำนวณ Kinematics (m/s)
  float target_v_l = linear - (angular * wheel_base / 2.0f);
  float target_v_r = linear + (angular * wheel_base / 2.0f);

  // 4. แปลงเป็น RPS และ PWM
  const float MAX_RPS = 2.0f; 
  float target_rps_l = target_v_l / circumference;
  float target_rps_r = target_v_r / circumference;
  
  float pwm_l = (target_rps_l / MAX_RPS) * 255.0f;
  float pwm_r = (target_rps_r / MAX_RPS) * 255.0f;

  // 5. สั่งงานมอเตอร์ (คูณ -1.0 ตามทิศทาง Hardware)
  Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, pwm_l * -1.0f);
  Motor_drive(WheelmotorRight_R, WheelmotorRight_L, pwm_r * -1.0f);

  // 6. ส่งค่าข้อมูลปัจจุบันกลับไป ROS เพื่อ Debug
  msg_vel_out.data.data[0] = pwm_l;
  msg_vel_out.data.data[1] = pwm_r;
  msg_setpoint.data.data[0] = target_rps_l;
  msg_setpoint.data.data[1] = target_rps_r;
  
  RCSOFTCHECK(rcl_publish(&pub_vel_out, &msg_vel_out, NULL));
}

