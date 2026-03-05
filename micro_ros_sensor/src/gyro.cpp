#include "main.h"

// ตัวแปรเก็บค่ามุม
float angleBodyX, angleBodyY, angleBodyZ;
float anglePlatformX, anglePlatformY, anglePlatformZ;
unsigned long last_gyro_time = 0;
float alpha_lpf = 0.15f;
// ตัวแปร Global
Adafruit_MPU6050 mpuBody;
Adafruit_MPU6050 mpuPlatform;
// กำหนดขนาดของ Window สำหรับ Median Filter
#define WINDOW_SIZE 5

// Buffer สำหรับเก็บค่าย้อนหลัง (Body)
float bodyAX_buf[WINDOW_SIZE], bodyAY_buf[WINDOW_SIZE], bodyAZ_buf[WINDOW_SIZE];
// Buffer สำหรับเก็บค่าย้อนหลัง (Platform)
float platAX_buf[WINDOW_SIZE], platAY_buf[WINDOW_SIZE], platAZ_buf[WINDOW_SIZE];

int buf_idx = 0; // ตัวชี้ตำแหน่ง Array

void Gyro() {
    sensors_event_t a, g, temp;
    float dt = (millis() - last_gyro_time) / 1000.0f;
    last_gyro_time = millis();

    // --- [อ่าน Body IMU - TCA 0] ---
    tcaSelect(0);
    mpuBody.getEvent(&a, &g, &temp);
    
    // เก็บค่าลง Buffer
    bodyAX_buf[buf_idx] = a.acceleration.x;
    bodyAY_buf[buf_idx] = a.acceleration.y;
    bodyAZ_buf[buf_idx] = a.acceleration.z;

    // หาค่า Median ของ Accelerometer ก่อนคำนวณมุม
    float fAX = medianFilter(bodyAX_buf, WINDOW_SIZE);
    float fAY = medianFilter(bodyAY_buf, WINDOW_SIZE);
    float fAZ = medianFilter(bodyAZ_buf, WINDOW_SIZE);

    // คำนวณมุมด้วย Complementary Filter (ใช้ค่าที่ผ่าน Median แล้ว)
    float rawAccAngleX = atan2(fAY, fAZ) * 180 / PI;
    float rawAccAngleY = atan2(-fAX, sqrt(pow(fAY, 2) + pow(fAZ, 2))) * 180 / PI;
    
    float currentAngleBodyX = 0.96f * (angleBodyX + g.gyro.x * dt) + 0.04f * rawAccAngleX;
    float currentAngleBodyY = 0.96f * (angleBodyY + g.gyro.y * dt) + 0.04f * rawAccAngleY;

    // ตบท้ายด้วย Low-pass Filter เพื่อความนิ่งสุดๆ
    angleBodyX = lowpassFilter(currentAngleBodyX, angleBodyX, 0.15f);
    angleBodyY = lowpassFilter(currentAngleBodyY, angleBodyY, 0.15f);

    // --- [อ่าน Platform IMU - TCA 1] ---
    tcaSelect(1);
    mpuPlatform.getEvent(&a, &g, &temp);
    
    platAX_buf[buf_idx] = a.acceleration.x;
    platAY_buf[buf_idx] = a.acceleration.y;
    platAZ_buf[buf_idx] = a.acceleration.z;

    float fPAX = medianFilter(platAX_buf, WINDOW_SIZE);
    float fPAY = medianFilter(platAY_buf, WINDOW_SIZE);
    float fPAZ = medianFilter(platAZ_buf, WINDOW_SIZE);

    float rawAccAnglePX = atan2(fPAY, fPAZ) * 180 / PI;
    float rawAccAnglePY = atan2(-fPAX, sqrt(pow(fPAY, 2) + pow(fPAZ, 2))) * 180 / PI;

    float currentAnglePlatformX = 0.96f * (anglePlatformX + g.gyro.x * dt) + 0.04f * rawAccAnglePX;
    float currentAnglePlatformY = 0.96f * (anglePlatformY + g.gyro.y * dt) + 0.04f * rawAccAnglePY;

    anglePlatformX = lowpassFilter(currentAnglePlatformX, anglePlatformX, 0.15f);
    anglePlatformY = lowpassFilter(currentAnglePlatformY, anglePlatformY, 0.15f);

    // อัปเดต Index สำหรับรอบถัดไป
    buf_idx = (buf_idx + 1) % WINDOW_SIZE;

    // เก็บค่าลงโครงสร้างข้อมูลของคุณ
    msg_imu_body.x = roundf(angleBodyX * 100.0f) / 100.0f;
    msg_imu_body.y = roundf(angleBodyY * 100.0f) / 100.0f;
    msg_imu_platform.x = roundf(anglePlatformX * 100.0f) / 100.0f;
    msg_imu_platform.y = roundf(anglePlatformY * 100.0f) / 100.0f;
}