#include "main.h"

Adafruit_MPU6050 mpuBody;
Adafruit_MPU6050 mpuPlatform;

// --- [ ค่าคงที่สำหรับการปรับจูน ] ---
#define GYRO_CAL_SAMPLES 200 
#define ALPHA 0.1f  // ค่า Filter (0.1 = นิ่งและตอบสนองดี)

// --- [ ตัวแปรเก็บค่ามุม (Filtered) ] ---
float angleBodyX = 0, angleBodyY = 0;
float anglePlatformX = 0, anglePlatformY = 0;
float gyroRateY_filtered = 0, gyroRatePY_filtered = 0;

// --- [ ตัวแปรสำหรับ Calibration (Offset/Bias) ] ---
// สำหรับ Gyro (ความเร็วเชิงมุม)
float biasBodyX = 0, biasBodyY = 0;
float biasPlatX = 0, biasPlatY = 0;
// สำหรับ Accelerometer (มุมเอียงเริ่มต้น)
float offsetBodyX = 0, offsetBodyY = 0;
float offsetPlatX = 0, offsetPlatY = 0;

// --- [ ฟังก์ชัน Set ศูนย์ (Calibration) ] ---
void calibrateSensors() {
    Serial.println("Calibrating IMUs... Keep it STEADY!");
    sensors_event_t a, g, temp;
    
    float sumBX = 0, sumBY = 0, sumPX = 0, sumPY = 0;
    float sumAngBX = 0, sumAngBY = 0, sumAngPX = 0, sumAngPY = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        // --- อ่านค่าจาก Body (TCA 0) ---
        tcaSelect(0);
        mpuBody.getEvent(&a, &g, &temp);
        sumBX += g.gyro.x; 
        sumBY += g.gyro.y;
        sumAngBX += atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
        sumAngBY += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;

        // --- อ่านค่าจาก Platform (TCA 1) ---
        tcaSelect(1);
        mpuPlatform.getEvent(&a, &g, &temp);
        sumPX += g.gyro.x; 
        sumPY += g.gyro.y;
        sumAngPX += atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
        sumAngPY += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
        
        delay(5); // หน่วงเวลาเล็กน้อยเพื่อให้ได้ค่าที่ต่างกันในแต่ละรอบ
    }

    // คำนวณค่าเฉลี่ยเพื่อใช้เป็นค่า "ศูนย์"
    biasBodyX = sumBX / (float)GYRO_CAL_SAMPLES;
    biasBodyY = sumBY / (float)GYRO_CAL_SAMPLES;
    biasPlatX = sumPX / (float)GYRO_CAL_SAMPLES;
    biasPlatY = sumPY / (float)GYRO_CAL_SAMPLES;

    offsetBodyX = sumAngBX / (float)GYRO_CAL_SAMPLES;
    offsetBodyY = sumAngBY / (float)GYRO_CAL_SAMPLES;
    offsetPlatX = sumAngPX / (float)GYRO_CAL_SAMPLES;
    offsetPlatY = sumAngPY / (float)GYRO_CAL_SAMPLES;

    // Reset ค่าในตัวแปรหลักให้เป็น 0 ทันทีหลัง Calibrate
    angleBodyX = 0; angleBodyY = 0;
    anglePlatformX = 0; anglePlatformY = 0;
    gyroRateY_filtered = 0; gyroRatePY_filtered = 0;

    Serial.println("Calibration Done. System Zeroed.");
}

// --- [ ฟังก์ชันอ่านค่าและประมวลผล ] ---
void Gyro() {
    sensors_event_t a, g, temp;
    
    // --- [ 1. จัดการ BODY IMU (TCA 0) ] ---
    tcaSelect(0);
    mpuBody.getEvent(&a, &g, &temp);

    // คำนวณค่า Raw และหักลบค่า Offset (เพื่อให้เริ่มที่ 0)
    float rawBodyX = (atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI) - offsetBodyX;
    float rawBodyY = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI) - offsetBodyY;
    float rawGyroRateY = (g.gyro.y - biasBodyY) * 180.0f / PI;

    // กรองสัญญาณรบกวน (Low-pass Filter)
    angleBodyX = (ALPHA * rawBodyX) + ((1.0f - ALPHA) * angleBodyX);
    angleBodyY = (ALPHA * rawBodyY) + ((1.0f - ALPHA) * angleBodyY);
    gyroRateY_filtered = (ALPHA * rawGyroRateY) + ((1.0f - ALPHA) * gyroRateY_filtered);

    // --- [ 2. จัดการ PLATFORM IMU (TCA 1) ] ---
    tcaSelect(1);
    mpuPlatform.getEvent(&a, &g, &temp);

    // คำนวณค่า Raw และหักลบค่า Offset
    float rawPlatX = (atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI) - offsetPlatX;
    float rawPlatY = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI) - offsetPlatY;
    float rawGyroRatePY = (g.gyro.y - biasPlatY) * 180.0f / PI;

    // กรองสัญญาณรบกวน (Low-pass Filter)
    anglePlatformX = (ALPHA * rawPlatX) + ((1.0f - ALPHA) * anglePlatformX);
    anglePlatformY = (ALPHA * rawPlatY) + ((1.0f - ALPHA) * anglePlatformY);
    gyroRatePY_filtered = (ALPHA * rawGyroRatePY) + ((1.0f - ALPHA) * gyroRatePY_filtered);

    // --- [ 3. ส่งข้อมูลออก (Mapping Index) ] ---
    msg_sensors.data.data[0] = roundf(angleBodyX * 100.0f) / 100.0f;
    msg_sensors.data.data[1] = roundf(angleBodyY * 100.0f) / 100.0f;
    msg_sensors.data.data[2] = roundf(anglePlatformX * 100.0f) / 100.0f;
    msg_sensors.data.data[3] = roundf(anglePlatformY * 100.0f) / 100.0f;
    
    // Index 5 และ 6 สำหรับความเร็วการเอียง (Angular Velocity)
    msg_sensors.data.data[5] = roundf(gyroRateY_filtered * 100.0f) / 100.0f;
    msg_sensors.data.data[6] = roundf(gyroRatePY_filtered * 100.0f) / 100.0f;
}