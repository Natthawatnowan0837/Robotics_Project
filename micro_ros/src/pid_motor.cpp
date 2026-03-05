
#include "main.h"

// ตัวแปรสำหรับ PID
float L_Wheel_vel, L_Wheel_Setpoint, L_Wheel_Input, L_Wheel_Output;
float R_Wheel_vel, R_Wheel_Setpoint, R_Wheel_Input, R_Wheel_Output;

// ค่า Gain (ปรับตามหุ่นยนต์ของคุณ)
float L_Kp = 13.0, L_Ki = 0.0, L_Kd = 0.0;
float R_Kp = 13.2, R_Ki = 0.0, R_Kd = 0.0;
// สร้าง Object สำหรับล้อซ้ายและขวา
QuickPID L_wheel_PID(&L_Wheel_Input, &L_Wheel_Output, &L_Wheel_Setpoint);
QuickPID R_wheel_PID(&R_Wheel_Input, &R_Wheel_Output, &R_Wheel_Setpoint);

// ฟังก์ชันขับมอเตอร์ (รองรับค่าบวก/ลบ)
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
  // ตั้งค่า Tuning แยกกัน
  L_wheel_PID.SetTunings(L_Kp, L_Ki, L_Kd);
  R_wheel_PID.SetTunings(R_Kp, R_Ki, R_Kd);

  // ตั้งค่า Output Limits (ปกติจะเท่ากันคือ 0-255 หรือ -255 ถึง 255)
  L_wheel_PID.SetOutputLimits(-255, 255);
  R_wheel_PID.SetOutputLimits(-255, 255);

  // เปิดใช้งาน PID
  L_wheel_PID.SetMode(L_wheel_PID.Control::automatic);
  R_wheel_PID.SetMode(R_wheel_PID.Control::automatic);
}

void pid_motor(float linear, float angular, float enc_left, float enc_right) {
  // --- Parameters (ปรับตามโครงสร้างจริงของหุ่นยนต์) ---
  float wheel_base = 0.7;      // ระยะห่างระหว่างล้อ (เมตร)
  float wheel_diameter = 0.15; // เส้นผ่านศูนย์กลางล้อ (เมตร)
  float circumference = wheel_diameter * PI;

  // 1. Kinematics: คำนวณความเร็วเป้าหมาย (m/s) ของแต่ละล้อ
  L_Wheel_vel = linear - (angular * wheel_base / 2.0);
  R_Wheel_vel = linear + (angular * wheel_base / 2.0);

  // 2. แปลงความเร็ว (m/s) เป็น Setpoint (RPS) เพื่อให้ตรงกับหน่วยของ Encoder
  // RPS = (m/s) / (เมตร/รอบ)
  L_Wheel_Setpoint = L_Wheel_vel / circumference;
  R_Wheel_Setpoint = R_Wheel_vel / circumference;

  // 3. อัปเดตค่า Input จาก Encoder (สมมติว่า enc_left/right คือค่า RPS ปัจจุบัน)
  L_Wheel_Input = enc_left; 
  R_Wheel_Input = enc_right; 

  // 4. คำนวณ PID (QuickPID จะคำนวณ Output ในช่วง -255 ถึง 255 ตามที่ตั้งค่าไว้)
  L_wheel_PID.Compute();
  R_wheel_PID.Compute();

  // 5. สั่งขับมอเตอร์ 
  // หมายเหตุ: คูณ -1.0 หากทิศทางมอเตอร์สวนทางกับค่าความเร็ว
  Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, L_Wheel_Output*-1.0);
  Motor_drive(WheelmotorRight_R, WheelmotorRight_L, R_Wheel_Output*-1.0);

  // --- ส่วนการ Publish ข้อมูลเพื่อ Debug ---
  msg_vel_out.data.data[0] = (float)L_Wheel_Output;
  msg_vel_out.data.data[1] = (float)R_Wheel_Output;

  msg_setpoint.data.data[0] = (float)L_Wheel_Setpoint;
  msg_setpoint.data.data[1] = (float)R_Wheel_Setpoint;

  RCSOFTCHECK(rcl_publish(&pub_vel_out, &msg_vel_out, NULL));
  RCSOFTCHECK(rcl_publish(&pub_setpoint, &msg_setpoint, NULL));
}
