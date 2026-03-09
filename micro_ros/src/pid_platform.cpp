#include <QuickPID.h>
#include "main.h"

// --- ตั้งค่าขอบเขตความเร็ว (Tunable Parameters) ---
float PWM_MIN = 10.0;   // ค่าต่ำสุดที่ต้องการให้มอเตอร์เริ่มหมุน (ป้องกัน Deadzone)
float PWM_MAX = 50.0;  // ค่าสูงสุดที่ยอมให้ PID สั่ง (จำกัดความเร็ว)

// --- ตัวแปรสำหรับ Platform PID ---
float Platform_Setpoint = 0.0; 
float Platform_Input = 0.0;    
float Platform_Output = 0.0;   

float error_sensors = 3.0;
// ปรับ Gain: Kp, Ki, Kd
float P_Kp = 10.0, P_Ki = 0.0, P_Kd = 0.2; 

unsigned long last_pid_time = 0;
const unsigned long pid_interval = 20; 

QuickPID Platform_PID(&Platform_Input, &Platform_Output, &Platform_Setpoint);

// ฟังก์ชันขับมอเตอร์
void Platform_drive(int pinA, int pinB, float speed) {
    speed = constrain(speed, -255, 255);

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

void init_plateformPID() {
    Platform_PID.SetTunings(P_Kp, P_Ki, P_Kd);
    
    // ตั้ง Limit ของ PID ให้เท่ากับ PWM_MAX ที่เรากำหนดไว้
    Platform_PID.SetOutputLimits(-PWM_MAX, PWM_MAX); 
    
    // REVERSE: เมื่อ Input ลดลง Output จะเพิ่มขึ้น (ปรับตามหน้างาน)
    Platform_PID.SetControllerDirection(QuickPID::Action::reverse); 
    
    Platform_PID.SetMode(QuickPID::Control::automatic);
    Platform_PID.SetSampleTimeUs(5000);
}

void pid_plateform(float platform_y, float hall_effect, float omega_platform_y) {
    unsigned long now = millis();
    if (now - last_pid_time < pid_interval) return;
    last_pid_time = now;

    // --- ส่วนการจัดการ Input และ Deadband ---
    float process_input = platform_y * -1.0;
    
    // ถ้าเอียงน้อยกว่า 3 องศา ให้ถือว่าตรง (ป้องกันมอเตอร์ครางจี๊ดๆ ตอนอยู่นิ่ง)
    if (platform_y > -error_sensors && platform_y < error_sensors) {
        process_input = Platform_Setpoint; 
    }
    
    Platform_Input = process_input;

    // --- คำนวณ PID ---
    Platform_PID.Compute();      
    float pid_out = Platform_Output; 
    float target_pwm = 0;

    // --- ส่วนการจัดการ PWM_MIN และ PWM_MAX (Deadzone Compensation) ---
    // ใช้การ Map เพื่อให้ค่าจาก 0 ถึง MAX กลายเป็น MIN ถึง MAX แบบ Linear
    if (pid_out > 0.1) {
        target_pwm = map(pid_out * 100, 0, PWM_MAX * 100, PWM_MIN * 100, PWM_MAX * 100) / 100.0;
    } else if (pid_out < -0.1) {
        target_pwm = map(pid_out * 100, -PWM_MAX * 100, 0, -PWM_MAX * 100, -PWM_MIN * 100) / 100.0;
    } else {
        target_pwm = 0;
    }

    // Safety Logic (Hall Effect Sensor)
    // ถ้าเซนเซอร์ทำงาน (1.0) และมอเตอร์กำลังพยายามหมุนไปทิศที่ติดลบ ให้หยุด
    if (hall_effect == 1.0 && target_pwm < 0) {
        target_pwm = 0; 
    }

    // สั่งงานมอเตอร์ (คูณ -1.0 หากทิศทางตรงข้ามกับที่ต้องการ)
    Platform_drive(PlatformLeft_R, PlatformLeft_L, target_pwm * -1.0);
    Platform_drive(PlatformRight_R, PlatformRight_L, target_pwm * -1.0); 

    // Debug ข้อมูลส่งไป micro-ROS
    msg_pub_balance.data.data[0] = platform_y;
    msg_pub_balance.data.data[1] = target_pwm; 
    msg_pub_balance.data.data[2] = hall_effect;
    msg_pub_balance.data.data[3] = omega_platform_y;
    RCSOFTCHECK(rcl_publish(&pub_balance, &msg_pub_balance, NULL));
}