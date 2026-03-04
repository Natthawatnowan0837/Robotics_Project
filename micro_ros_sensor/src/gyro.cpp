#include "main.h"

// --- ตั้งค่า Filter ---
// ค่า ALPHA ยิ่งมากยิ่งเชื่อ Gyro (ลื่นแต่ไหล), ยิ่งน้อยยิ่งเชื่อ Accel (นิ่งแต่ช้าและสั่นตามแรงสั่นสะเทือน)
#define FILTER_ALPHA 0.98 
#define GYRO_THRESHOLD 0.05 // ตัด Noise ที่ต่ำกว่าค่านี้

Adafruit_MPU6050 mpuBody;
Adafruit_MPU6050 mpuPlatform;

// ตัวแปรสะสมมุม
float angleBodyX = 0, angleBodyY = 0, angleBodyZ = 0;
float anglePlatformX = 0, anglePlatformY = 0, anglePlatformZ = 0;

// ตัวแปรเก็บค่า Offset (เพื่อแก้ปัญหา Gyro ไหลตอนอยู่นิ่ง)
float offBodyGX = 0, offBodyGY = 0, offBodyGZ = 0;
float offPlatGX = 0, offPlatGY = 0, offPlatGZ = 0;

unsigned long last_gyro_time = 0;

void Gyro() {
    unsigned long current_time = millis();
    if (last_gyro_time == 0) {
        last_gyro_time = current_time;
        return;
    }
    float dt = (current_time - last_gyro_time) / 1000.0;
    last_gyro_time = current_time;

    sensors_event_t a, g, temp;

    // --- ประมวลผล IMU Body (CH 0) ---
    tcaSelect(0);
    if (mpuBody.getEvent(&a, &g, &temp)) {
        // 1. ลบค่า Offset และแปลงเป็น Degree
        float gx = (g.gyro.x * RAD_TO_DEG) - offBodyGX;
        float gy = (g.gyro.y * RAD_TO_DEG) - offBodyGY;
        float gz = (g.gyro.z * RAD_TO_DEG) - offBodyGZ;

        // 2. คำนวณมุมจาก Accelerometer (เฉพาะ X และ Y เพราะ Z คำนวณไม่ได้)
        float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
        float accAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;

        // 3. Complementary Filter (ผสมผสาน Gyro + Accel)
        angleBodyX = FILTER_ALPHA * (angleBodyX + gx * dt) + (1.0 - FILTER_ALPHA) * accAngleX;
        angleBodyY = FILTER_ALPHA * (angleBodyY + gy * dt) + (1.0 - FILTER_ALPHA) * accAngleY;
        
        // 4. แกน Z (Yaw) - ใช้ Gyro integration อย่างเดียว (เพราะ Accel ช่วยไม่ได้)
        if (abs(gz) > GYRO_THRESHOLD) angleBodyZ += gz * dt;

        msg_imu_body.x = angleBodyX;
        msg_imu_body.y = angleBodyY;
        msg_imu_body.z = angleBodyZ;
    }

    // --- ประมวลผล IMU Platform (CH 1) ---
    tcaSelect(1);
    if (mpuPlatform.getEvent(&a, &g, &temp)) {
        float gx = (g.gyro.x * RAD_TO_DEG) - offPlatGX;
        float gy = (g.gyro.y * RAD_TO_DEG) - offPlatGY;
        float gz = (g.gyro.z * RAD_TO_DEG) - offPlatGZ;

        float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
        float accAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;

        anglePlatformX = FILTER_ALPHA * (anglePlatformX + gx * dt) + (1.0 - FILTER_ALPHA) * accAngleX;
        anglePlatformY = FILTER_ALPHA * (anglePlatformY + gy * dt) + (1.0 - FILTER_ALPHA) * accAngleY;
        
        if (abs(gz) > GYRO_THRESHOLD) anglePlatformZ += gz * dt;

        msg_imu_platform.x = anglePlatformX;
        msg_imu_platform.y = anglePlatformY;
        msg_imu_platform.z = anglePlatformZ;
    }
}

void initIMUs() {
    Serial.println("IMU Initialization...");
    
    // ตั้งค่าเบื้องต้น
    tcaSelect(0);
    if (!mpuBody.begin()) Serial.println("Body IMU Connect Failed");
    mpuBody.setGyroRange(MPU6050_RANGE_250_DEG);
    mpuBody.setFilterBandwidth(MPU6050_BAND_21_HZ); // เพิ่ม Hardware Filter ลด Noise

    tcaSelect(1);
    if (!mpuPlatform.begin()) Serial.println("Platform IMU Connect Failed");
    mpuPlatform.setGyroRange(MPU6050_RANGE_250_DEG);
    mpuPlatform.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // --- ขั้นตอน Calibration (สำคัญมาก!) ---
    // ต้องวางเซนเซอร์ให้นิ่งที่สุดขณะเปิดเครื่อง
    Serial.println("Calibrating... Keep it still for 2 seconds");
    int samples = 200;
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        tcaSelect(0); mpuBody.getEvent(&a, &g, &temp);
        offBodyGX += (g.gyro.x * RAD_TO_DEG);
        offBodyGY += (g.gyro.y * RAD_TO_DEG);
        offBodyGZ += (g.gyro.z * RAD_TO_DEG);

        tcaSelect(1); mpuPlatform.getEvent(&a, &g, &temp);
        offPlatGX += (g.gyro.x * RAD_TO_DEG);
        offPlatGY += (g.gyro.y * RAD_TO_DEG);
        offPlatGZ += (g.gyro.z * RAD_TO_DEG);
        delay(5);
    }
    offBodyGX /= samples; offBodyGY /= samples; offBodyGZ /= samples;
    offPlatGX /= samples; offPlatGY /= samples; offPlatGZ /= samples;
    
    Serial.println("Calibration Done!");
    last_gyro_time = millis();
}