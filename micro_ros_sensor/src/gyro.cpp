#include "main.h"

// --- [ การตั้งค่าตัวแปร IMU ] ---
Adafruit_MPU6050 mpuBody;
Adafruit_MPU6050 mpuPlatform;

#define GYRO_CAL_SAMPLES 200 
#define ALPHA 0.1f  // Low-pass Filter

// --- [ ตัวแปรเก็บค่ามุม Platform (Filtered) ] ---
float anglePlatformX = 0, anglePlatformY = 0;

// --- [ ตัวแปรสำหรับ Calibration (Offset/Bias) ] ---
float biasBodyX = 0, biasBodyY = 0, biasBodyZ = 0; 
float offsetPlatX = 0, offsetPlatY = 0;

// --- [ ฟังก์ชัน Set ศูนย์ (Calibration) ] ---
void calibrateSensors() {
    Serial.println("Calibrating IMUs... Keep it STEADY!");
    sensors_event_t a, g, temp;
    
    float sumBX = 0, sumBY = 0, sumBZ = 0;
    float sumAngPX = 0, sumAngPY = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        // --- 1. อ่านค่าจาก Body (TCA 0) เพื่อหา Bias ของ Gyro ทุกแกน ---
        tcaSelect(0);
        mpuBody.getEvent(&a, &g, &temp);
        sumBX += g.gyro.x; 
        sumBY += g.gyro.y;
        sumBZ += g.gyro.z;

        // --- 2. อ่านค่าจาก Platform (TCA 1) เพื่อหา Offset มุมเอียง ---
        tcaSelect(1);
        mpuPlatform.getEvent(&a, &g, &temp);
        sumAngPX += atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
        sumAngPY += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
        
        delay(5);
    }

    // คำนวณค่าเฉลี่ย Bias
    biasBodyX = sumBX / (float)GYRO_CAL_SAMPLES;
    biasBodyY = sumBY / (float)GYRO_CAL_SAMPLES;
    biasBodyZ = sumBZ / (float)GYRO_CAL_SAMPLES;

    offsetPlatX = sumAngPX / (float)GYRO_CAL_SAMPLES;
    offsetPlatY = sumAngPY / (float)GYRO_CAL_SAMPLES;

    anglePlatformX = 0; anglePlatformY = 0;
    Serial.println("Calibration Done. System Zeroed.");
}

// --- [ ฟังก์ชันหลักสำหรับอ่านและส่งค่า ] ---
void Gyro() {
    sensors_event_t a, g, temp;
    
    // --- [ 1. ดึงข้อมูล BODY (TCA 0) ] ---
    tcaSelect(0);
    mpuBody.getEvent(&a, &g, &temp);
    
    float body_accel_x = a.acceleration.x;
    float body_accel_y = a.acceleration.y;
    float body_accel_z = a.acceleration.z;
    float body_gyro_x  = g.gyro.x - biasBodyX; // เพิ่มแกน X
    float body_gyro_y  = g.gyro.y - biasBodyY;
    float body_gyro_z  = g.gyro.z - biasBodyZ;

    // --- [ 2. ดึงข้อมูล PLATFORM (TCA 1) ] ---
    tcaSelect(1);
    mpuPlatform.getEvent(&a, &g, &temp);

    float rawPlatX = (atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI) - offsetPlatX;
    float rawPlatY = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI) - offsetPlatY;

    // Low-pass Filter
    anglePlatformX = (ALPHA * rawPlatX) + ((1.0f - ALPHA) * anglePlatformX);
    anglePlatformY = (ALPHA * rawPlatY) + ((1.0f - ALPHA) * anglePlatformY);

    // --- [ 3. Mapping Index (0-7) ] ---
    msg_sensors.data.data[0] = body_accel_x;                             // Accel X
    msg_sensors.data.data[1] = body_accel_y;                             // Accel Y
    msg_sensors.data.data[2] = body_accel_z;                             // Accel Z
    msg_sensors.data.data[3] = body_gyro_x;                              // Gyro X (rad/s)
    msg_sensors.data.data[4] = body_gyro_y;                              // Gyro Y (rad/s)
    msg_sensors.data.data[5] = body_gyro_z;                              // Gyro Z (rad/s)
    msg_sensors.data.data[6] = roundf(anglePlatformX * 100.0f) / 100.0f; // Platform X (deg)
    msg_sensors.data.data[7] = roundf(anglePlatformY * 100.0f) / 100.0f; // Platform Y (deg)
}