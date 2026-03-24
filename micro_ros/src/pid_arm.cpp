#include <QuickPID.h>
#include "main.h"

// --- ตัวแปรสำหรับ Sync PID ---
float sync_Setpoint = 0;    // เป้าหมายคือความต่างต้องเป็น 0
float sync_Input = 0;       // input คือ (motorL - motorR)
float sync_Output = 0;      // ค่าที่จะเอาไปชดเชย PWM

// ปรับ Gain ตามความเหมาะสม (เริ่มจาก Kp น้อยๆ ก่อน)
float s_Kp = 12.0, s_Ki = 0.5, s_Kd = 1.0; 

// สร้าง Object สำหรับ PID
QuickPID Sync_PID(&sync_Input, &sync_Output, &sync_Setpoint);

static unsigned long last_pid_time = 0;
static unsigned long pid_interval = 20; 

// --- ฟังก์ชัน Setup ---
void init_armPID() {
    Sync_PID.SetTunings(pid_arm_parameters[0], pid_arm_parameters[1], pid_arm_parameters[2]);
    
    // จำกัดค่าชดเชยไม่ให้เกิน +/- 60 PWM เพื่อความปลอดภัย
    Sync_PID.SetOutputLimits(-60, 60); 
    
    // โหมด Automatic และตั้งค่า Sample Time (ไมโครวินาที)
    Sync_PID.SetMode(QuickPID::Control::automatic);
    Sync_PID.SetSampleTimeUs(pid_interval * 1000); 
}

// --- ฟังก์ชันขับมอเตอร์ ---
void Arm_drive(int pinA, int pinB, float speed) {
    speed = constrain(speed, -255, 255);
    
    // Deadzone Compensation
    if (abs(speed) > 0.1 && abs(speed) < 25) {
        speed = (speed > 0) ? 25 : -25;
    }

    if (speed > 0) {
        analogWrite(pinA, (int)speed);
        analogWrite(pinB, 0);
    } else if (speed < 0) {
        analogWrite(pinA, 0);
        analogWrite(pinB, (int)-speed);
    } else {
        analogWrite(pinA, 0);
        analogWrite(pinB, 0);
    }
}

// --- ฟังก์ชันหลักที่เรียกใช้ใน Loop ---
void pid_arm(float arm_control, float motorArm_L, float motorArm_R) {
    unsigned long now = millis();
    if (now - last_pid_time < pid_interval) return;
    last_pid_time = now;

    // 1. คำนวณ Input สำหรับ PID
    sync_Input = motorArm_L - motorArm_R;

    // 2. ประมวลผล PID
    Sync_PID.Compute(); 

    // 3. คำนวณ Base PWM (จำกัดเพดานเพื่อเหลือ Headroom)
    float base_pwm = arm_control * 190.0f;

    // 4. ผสมสัญญาณ (Mixing)
    float final_pwm_L = base_pwm - sync_Output;
    float final_pwm_R = base_pwm + sync_Output;

    // 5. ปรับปรุงส่วน Idle: ถ้าไม่มี Input จากผู้ใช้ และความต่างน้อย ให้หยุดสนิท
    // เพื่อป้องกันมอเตอร์ครางจี๊ดๆ ตอนอยู่นิ่ง
    if (abs(arm_control) < 0.05 && abs(sync_Input) < 3.0) {
        final_pwm_L = 0;
        final_pwm_R = 0;
    }

    // 6. สั่งงาน Motor (ระวังเรื่อง Pin ทิศทางมอเตอร์แต่ละข้าง)
    Arm_drive(ArmLeft_R, ArmLeft_L, final_pwm_L);
    Arm_drive(ArmRight_R, ArmRight_L, final_pwm_R);

    // 7. Debug ข้อมูลส่งไป ROS
    msg_pub_stateArm.data.data[0] = sync_Input;   
    msg_pub_stateArm.data.data[1] = final_pwm_L; 
    msg_pub_stateArm.data.data[2] = final_pwm_R;
    RCSOFTCHECK(rcl_publish(&pub_stateArm, &msg_pub_stateArm, NULL));
}