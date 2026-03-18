#include "main.h"
#include <math.h>

// --- 1. ค่าคงที่และตัวแปรระบบ (ครบถ้วน) ---
const float WHEEL_DI = 0.15; 
const float WHEEL_BASE = 0.7;     

unsigned long last_time_motor = 0;
float last_angle_ml = 0, last_angle_mr = 0;
float buffer_l[MEDIAN_SIZE], buffer_r[MEDIAN_SIZE];
float filtered_rps_l = 0, filtered_rps_r = 0;
float f_linear_vel = 0, f_angular_vel = 0;
float odom_x = 0, odom_y = 0, odom_theta = 0;

AS5600 as5600_motor(&Wire); 

void Encoder_motor() {
    // --- 2. การจัดการเวลา (Time Delta) ---
    unsigned long current_time = micros();
    float dt = (float)(current_time - last_time_motor) / 1000000.0f;
    if (dt < 0.005f) return; 

    const float RAW_TO_DEG = (360.0f / 4096.0f) * -1.0f; // ปรับทิศทางตามที่ต้องการ
    const float alpha_wheel = 0.4f; 
    const float alpha_vel = 0.3f;   

    // --- 3. อ่านค่าจาก Sensor (TCA9548A Select) ---
    tcaSelect(2);
    float current_angle_l = as5600_motor.readAngle() * RAW_TO_DEG;
    tcaSelect(3);
    float current_angle_r = as5600_motor.readAngle() * RAW_TO_DEG;

    // --- 4. คำนวณความต่างองศาและ Rollover ---
    float delta_l_deg = current_angle_l - last_angle_ml;
    float delta_r_deg = current_angle_r - last_angle_mr;

    if (delta_l_deg > 180)  delta_l_deg -= 360;
    else if (delta_l_deg < -180) delta_l_deg += 360;
    if (delta_r_deg > 180)  delta_r_deg -= 360;
    else if (delta_r_deg < -180) delta_r_deg += 360;

    // --- 5. คำนวณ RPS (หน่วยรอบต่อวินาที) ---
    float raw_rps_l = (delta_l_deg / 360.0f) / dt;
    float raw_rps_r = (delta_r_deg / 360.0f) / dt;

    // --- 6. Median Filter (ลด Noise) ---
    for (int i = MEDIAN_SIZE - 1; i > 0; i--) {
        buffer_l[i] = buffer_l[i - 1];
        buffer_r[i] = buffer_r[i - 1];
    }
    buffer_l[0] = raw_rps_l;
    buffer_r[0] = raw_rps_r;

    float median_l = medianFilter(buffer_l, MEDIAN_SIZE);
    float median_r = medianFilter(buffer_r, MEDIAN_SIZE);

    // --- 7. Low-pass Filter (RPS) ---
    filtered_rps_l = (alpha_wheel * median_l) + ((1.0f - alpha_wheel) * filtered_rps_l);
    filtered_rps_r = (alpha_wheel * median_r) + ((1.0f - alpha_wheel) * filtered_rps_r);

    // เตรียมค่า RPS ส่งไปที่ data[0], [1]
    float out_rps_l = (abs(filtered_rps_l) < 0.01f) ? 0.0f : roundf(filtered_rps_l * 100.0f) / 100.0f;
    float out_rps_r = (abs(filtered_rps_r) < 0.01f) ? 0.0f : roundf(filtered_rps_r * 100.0f) / 100.0f;

    // --- 8. แปลงเป็นหน่วยเมตร (m/s) และคำนวณความเร็วรวม ---
    float v_l = filtered_rps_l * (M_PI * WHEEL_DI);
    float v_r = filtered_rps_r * (M_PI * WHEEL_DI);

    float raw_linear_vel = (v_r + v_l) / 2.0f;
    float raw_angular_vel = (v_r - v_l) / WHEEL_BASE;

    f_linear_vel = (alpha_vel * raw_linear_vel) + ((1.0f - alpha_vel) * f_linear_vel);
    f_angular_vel = (alpha_vel * raw_angular_vel) + ((1.0f - alpha_vel) * f_angular_vel);

    // --- 9. คำนวณ Odometry ---
    float d_center = raw_linear_vel * dt;      
    float d_theta = raw_angular_vel * dt; 

    odom_theta += d_theta;
    if (odom_theta > M_PI)  odom_theta -= 2.0 * M_PI;
    if (odom_theta < -M_PI) odom_theta += 2.0 * M_PI;
    odom_x += d_center * cos(odom_theta);
    odom_y += d_center * sin(odom_theta);

    // --- 10. ส่งค่าเข้า Message Structure (data[0] ถึง [5]) ---
    msg_motor.data.data[0] = out_rps_l*-1; 
    msg_motor.data.data[1] = out_rps_r*-1; 
    msg_motor.data.data[2] = (abs(f_linear_vel) < 0.005f) ? 0.0f : f_linear_vel;
    msg_motor.data.data[3] = (abs(f_angular_vel) < 0.005f) ? 0.0f : f_angular_vel;
    msg_motor.data.data[4] = d_center;
    msg_motor.data.data[5] = d_theta;

    // --- 11. อัปเดตสถานะสำหรับ Loop ถัดไป ---
    last_angle_ml = current_angle_l;
    last_angle_mr = current_angle_r;
    last_time_motor = current_time;
}