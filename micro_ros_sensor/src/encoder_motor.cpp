#include "main.h"

// ประกาศตัวแปรคำนวณที่ใช้เฉพาะในไฟล์นี้
unsigned long last_time_motor = 0;
float last_angle_ml = 0, last_angle_mr = 0;
float buffer_l[MEDIAN_SIZE], buffer_r[MEDIAN_SIZE];
float filtered_rps_l = 0, filtered_rps_r = 0;
bool encoder_online[2] = {true, true};

// สร้าง Object as5600_motor ให้ตรงกับที่ main.cpp เรียกใช้
AS5600 as5600_motor(&Wire); 



void Encoder_motor() {
    unsigned long current_time = micros();
    float dt = (float)(current_time - last_time_motor) / 1000000.0;
    
    if (dt < 0.005) return; 

    // ปรับทิศทางที่นี่: ถ้าเดิมเดินหน้าเป็นลบ ให้ใส่ -1.0 เข้าไป
    const float RAW_TO_DEG = (360.0 / 4096.0) * -1.0; 
    const float alpha = 0.5;

    // --- ล้อซ้าย (Channel 2) ---
    tcaSelect(2);
    float current_angle_l = as5600_motor.readAngle() * RAW_TO_DEG;
    float delta_l = current_angle_l - last_angle_ml;
    
    if (delta_l > 180) delta_l -= 360;
    else if (delta_l < -180) delta_l += 360;
    
    float raw_rps_l = (delta_l / 360.0) / dt;
    
    // Median + Alpha Filter
    for (int i = MEDIAN_SIZE - 1; i > 0; i--) buffer_l[i] = buffer_l[i - 1];
    buffer_l[0] = raw_rps_l;
    filtered_rps_l = (alpha * medianFilter(buffer_l, MEDIAN_SIZE)) + ((1.0 - alpha) * filtered_rps_l);
    
    // --- ล้อขวา (Channel 3) ---
    tcaSelect(3);
    float current_angle_r = as5600_motor.readAngle() * RAW_TO_DEG;
    float delta_r = current_angle_r - last_angle_mr;
    
    if (delta_r > 180) delta_r -= 360;
    else if (delta_r < -180) delta_r += 360;
    
    float raw_rps_r = (delta_r / 360.0) / dt;

    for (int i = MEDIAN_SIZE - 1; i > 0; i--) buffer_r[i] = buffer_r[i - 1];
    buffer_r[0] = raw_rps_r;
    filtered_rps_r = (alpha * medianFilter(buffer_r, MEDIAN_SIZE)) + ((1.0 - alpha) * filtered_rps_r);

    // --- การปัดทศนิยม 2 ตำแหน่ง และ Deadzone ---
    float out_l = (abs(filtered_rps_l) < 0.02) ? 0.00f : roundf(filtered_rps_l * 100.0f) / 100.0f;
    float out_r = (abs(filtered_rps_r) < 0.02) ? 0.00f : roundf(filtered_rps_r * 100.0f) / 100.0f;

    // ส่งค่าเข้า Message
    msg_motor.data.data[0] = out_l;
    msg_motor.data.data[1] = out_r;

    // เก็บค่าไว้ใช้ในรอบถัดไป
    last_angle_ml = current_angle_l;
    last_angle_mr = current_angle_r;
    last_time_motor = current_time;
}