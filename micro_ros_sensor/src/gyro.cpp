#include "main.h" 

// --- [ การตั้งค่าตัวแปร IMU ] ---
Adafruit_MPU6050 mpuBody;
Adafruit_MPU6050 mpuPlatform;

#define ALPHA_PLAT 0.1f       
#define COMP_FILTER_GAIN 0.96f // 0.96 เชื่อ Gyro, 0.04 เชื่อ Accel

// --- [ ตัวแปรเก็บค่ามุม ] ---
float bodyRoll = 0, bodyPitch = 0, bodyYaw = 0;
float anglePlatformY = 0; 

unsigned long lastTime = 0;

// *** หมายเหตุ: ยกเลิก calibrateSensors() แล้ว ***
// ระบบจะใช้ค่าจาก Accelerometer ตั้งต้นเป็นมุมปัจจุบันทันทีเมื่อเริ่มทำงาน

void Gyro() {
    sensors_event_t a, g, temp;
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f; 
    
    if (lastTime == 0 || dt <= 0 || dt > 0.5f) {
        dt = 0.01f; 
    }
    lastTime = currentTime;

    // --- [ 1. อ่านข้อมูลจาก Body MPU (TCA 0) ] ---
    tcaSelect(0);
    if (mpuBody.getEvent(&a, &g, &temp)) {
        
        // คำนวณมุมจาก Accelerometer (Radian)
        float accRollBodyRad  = atan2(a.acceleration.y, a.acceleration.z);
        float accPitchBodyRad = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));

        // ค่า Gyro ดิบ (rad/s)
        float gyroX_rads = g.gyro.x;
        float gyroY_rads = g.gyro.y;
        float gyroZ_rads = g.gyro.z;

        // Complementary Filter (หน่วย Radian)
        bodyRoll  = COMP_FILTER_GAIN * (bodyRoll + gyroX_rads * dt) + (1.0f - COMP_FILTER_GAIN) * accRollBodyRad;
        bodyPitch = COMP_FILTER_GAIN * (bodyPitch + gyroY_rads * dt) + (1.0f - COMP_FILTER_GAIN) * accPitchBodyRad;
        bodyYaw  += gyroZ_rads * dt; 

        // --- เพิ่มส่วนนี้: แปลง bodyPitch เป็นองศาสำหรับ angleBodyY ---
        float angleBodyY_deg = bodyPitch * 180.0f / PI;

        // --- [ ส่งข้อมูลลง msg_sensors ] ---
        msg_sensors.data.data[0] = bodyRoll;      // rad
        msg_sensors.data.data[1] = bodyPitch;     // rad
        msg_sensors.data.data[2] = bodyYaw;       // rad
        msg_sensors.data.data[3] = gyroX_rads;    // rad/s
        msg_sensors.data.data[4] = gyroY_rads;    // rad/s
        msg_sensors.data.data[5] = gyroZ_rads;    // rad/s
        
        // ใส่ค่ามุมของตัวหุ่น (หน่วยองศา) ในช่องที่ 6
        msg_sensors.data.data[6] = angleBodyY_deg; 
    }

    // --- [ 2. อ่านข้อมูลจาก Platform MPU (TCA 1) ] ---
    tcaSelect(1);
    if (mpuPlatform.getEvent(&a, &g, &temp)) {
        // คำนวณมุม Pitch ของ Platform (Radian)
        float accPitchPlatRad = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));
        
        // แปลงเป็นองศา
        float accPitchPlatDeg = accPitchPlatRad * 180.0f / PI;

        // Low-pass filter สำหรับ Platform
        anglePlatformY = (ALPHA_PLAT * accPitchPlatDeg) + ((1.0f - ALPHA_PLAT) * anglePlatformY);

        // ใส่ค่ามุมของฐานรอง (หน่วยองศา) ในช่องที่ 7
        msg_sensors.data.data[7] = anglePlatformY; 
    }
}