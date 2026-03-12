// #include <QuickPID.h>
// #include "main.h"

// // --- ตัวแปรสำหรับเก็บค่าความเร็วและ PID ---
// float L_Wheel_vel, L_Wheel_Setpoint, L_Wheel_Input, L_Wheel_Output;
// float R_Wheel_vel, R_Wheel_Setpoint, R_Wheel_Input, R_Wheel_Output;

// // ค่า Default (จะถูกเขียนทับทันทีที่มีข้อมูลจาก ROS 2)
// float L_Kp = 13.0, L_Ki = 0.0, L_Kd = 0.0;
// float R_Kp = 13.0, R_Ki = 0.0, R_Kd = 0.0;

// // สร้าง Object สำหรับ QuickPID
// QuickPID L_wheel_PID(&L_Wheel_Input, &L_Wheel_Output, &L_Wheel_Setpoint);
// QuickPID R_wheel_PID(&R_Wheel_Input, &R_Wheel_Output, &R_Wheel_Setpoint);

// /**
//  * ฟังก์ชันสั่งงานมอเตอร์ผ่าน PWM
//  */
// void Motor_drive(int pinA, int pinB, float speed) {
//   // จำกัดค่าไม่ให้เกินขอบเขต PWM (0-255)
//   speed = constrain(speed, -255, 255);
  
//   if (speed > 0) {
//     analogWrite(pinA, (int)speed);
//     analogWrite(pinB, 0);
//   } else if (speed < 0) {
//     analogWrite(pinA, 0);
//     analogWrite(pinB, (int)-speed);
//   } else {
//     analogWrite(pinA, 0);
//     analogWrite(pinB, 0);
//   }
// }
// void init_PID() {
//   // ตั้งค่า Tuning เริ่มต้น
//   L_wheel_PID.SetTunings(L_Kp, L_Ki, L_Kd);
//   R_wheel_PID.SetTunings(R_Kp, R_Ki, R_Kd);

//   // ตั้งขอบเขต Output ให้ตรงกับช่วง PWM
//   L_wheel_PID.SetOutputLimits(-255, 255);
//   R_wheel_PID.SetOutputLimits(-255, 255);

//   // เปิดใช้งานโหมดอัตโนมัติ
//   L_wheel_PID.SetMode(L_wheel_PID.Control::automatic);
//   R_wheel_PID.SetMode(R_wheel_PID.Control::automatic);
  
//   // ตั้งค่า Sample Time ให้ตรงกับ timer_callback (20ms)
//   L_wheel_PID.SetSampleTimeUs(20000); 
//   R_wheel_PID.SetSampleTimeUs(20000);
// }

// /**
//  * ฟังก์ชันหลักในการคำนวณ Differential Drive และ PID
//  */
// void pid_drive(float linear, float angular, float motorDrive_L, float motorDrive_R){
  
//   // 1. อัปเดตค่า PID Tuning ที่ได้รับมาจาก ROS 2 (ผ่าน Pointer)
//   // [0]=Kp, [1]=Ki, [2]=Kd
//   L_wheel_PID.SetTunings(pid_driveL_parameters[0], pid_driveL_parameters[1], pid_driveL_parameters[2]);
//   R_wheel_PID.SetTunings(pid_driveR_parameters[0], pid_driveR_parameters[1], pid_driveR_parameters[2]);
  
//   // 2. ข้อมูลทางกายภาพของหุ่นยนต์ (ปรับตามจริง)
//   float wheel_base = 0.7;      // ความกว้างระหว่างล้อ (เมตร)
//   float wheel_diameter = 0.15; // เส้นผ่านศูนย์กลางล้อ (เมตร)
//   float circumference = wheel_diameter * PI;

//   // 3. แปลงความเร็วหุ่นยนต์ (Linear/Angular) เป็นความเร็วแต่ละล้อ (m/s)
//   L_Wheel_vel = linear - (angular * wheel_base / 2.0);
//   R_Wheel_vel = linear + (angular * wheel_base / 2.0);

//   // 4. แปลงความเร็ว (m/s) เป็น Setpoint สำหรับ PID (เช่น รอบต่อวินาที หรือหน่วยที่สัมพันธ์กับ Encoder)
//   // หมายเหตุ: ตรงนี้ต้องสัมพันธ์กับหน่วยของ motorDrive_L/R ที่รับมาจาก Encoder
//   L_Wheel_Setpoint = L_Wheel_vel / circumference;
//   R_Wheel_Setpoint = R_Wheel_vel / circumference;

//   // 5. ป้อนค่าปัจจุบันจาก Sensor เข้าสู่ระบบ PID
//   L_Wheel_Input = motorDrive_L; 
//   R_Wheel_Input = motorDrive_R; 

//   // 6. ประมวลผล PID
//   L_wheel_PID.Compute();
//   R_wheel_PID.Compute();

//   // 7. ส่งคำสั่ง PWM ไปยังมอเตอร์
//   // คูณ -1.0 หากทิศทางมอเตอร์สวนทางกับค่าบวกของ PID
//   Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, L_Wheel_Output * -1.0);
//   Motor_drive(WheelmotorRight_R, WheelmotorRight_L, R_Wheel_Output * -1.0);

//   // 8. เตรียมข้อมูลส่งกลับไปยัง ROS 2 เพื่อทำกราฟวิเคราะห์ (Log)
//   msg_pub_drive.data.data[0] = (float)L_Wheel_Output;   // ส่งค่า PWM ที่ใช้
//   msg_pub_drive.data.data[1] = (float)R_Wheel_Output;
//   msg_pub_drive.data.data[2] = (float)L_Wheel_Setpoint; // ส่งค่าเป้าหมาย
//   msg_pub_drive.data.data[3] = (float)R_Wheel_Setpoint;

//   // ส่งข้อมูลออกทาง Topic "drive"
//   RCSOFTCHECK(rcl_publish(&pub_drive, &msg_pub_drive, NULL));
// }