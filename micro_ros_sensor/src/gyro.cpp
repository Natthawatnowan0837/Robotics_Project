#include "main.h" // ตรวจสอบว่ามี Library Adafruit_MPU6050, Adafruit_Sensor และ Wire

// --- [ การตั้งค่าตัวแปร IMU ] ---
Adafruit_MPU6050 mpuBody;
Adafruit_MPU6050 mpuPlatform;

#define GYRO_CAL_SAMPLES 200 
#define ALPHA_PLAT 0.1f       // Low-pass Filter สำหรับ Platform
#define COMP_FILTER_GAIN 0.96f // Complementary Filter (0.96 เชื่อ Gyro, 0.04 เชื่อ Accel)

// --- [ ตัวแปรเก็บค่ามุม (Filtered & Integration) ] ---
float bodyRoll = 0, bodyPitch = 0, bodyYaw = 0;
float anglePlatformX = 0, anglePlatformY = 0;

// --- [ ตัวแปรสำหรับ Calibration (Offset/Bias) ] ---
float biasBodyX = 0, biasBodyY = 0, biasBodyZ = 0; 
float offsetBodyRoll = 0, offsetBodyPitch = 0; // เพิ่มสำหรับ Body
float offsetPlatX = 0, offsetPlatY = 0;        // สำหรับ Platform

unsigned long lastTime = 0;

// --- [ ฟังก์ชัน Set ศูนย์ (Calibration) ] ---
void calibrateSensors() {
    Serial.println(">>> Calibrating ALL IMUs... Keep the robot STEADY! <<<");
    sensors_event_t a, g, temp;
    
    float sumBX = 0, sumBY = 0, sumBZ = 0;
    float sumBodyRoll = 0, sumBodyPitch = 0;
    float sumAngPX = 0, sumAngPY = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        // 1. อ่านค่าจาก Body (TCA 0)
        tcaSelect(0);
        mpuBody.getEvent(&a, &g, &temp);
        // เก็บค่า Gyro สำหรับ Bias
        sumBX += g.gyro.x; 
        sumBY += g.gyro.y;
        sumBZ += g.gyro.z;
        // เก็บค่า Accel สำหรับหาความเอียงเริ่มต้น (Static Offset)
        sumBodyRoll  += atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
        sumBodyPitch += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;

        // 2. อ่านค่าจาก Platform (TCA 1)
        tcaSelect(1);
        mpuPlatform.getEvent(&a, &g, &temp);
        sumAngPX += atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
        sumAngPY += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
        
        delay(5); // หน่วงเวลาสั้นๆ เพื่อให้ได้ข้อมูลที่หลากหลายขึ้น
    }

    // คำนวณค่าเฉลี่ย Bias ของ Gyro Body (หน่วย rad/s)
    biasBodyX = sumBX / (float)GYRO_CAL_SAMPLES;
    biasBodyY = sumBY / (float)GYRO_CAL_SAMPLES;
    biasBodyZ = sumBZ / (float)GYRO_CAL_SAMPLES;

    // คำนวณค่าเฉลี่ย Offset มุมของ Body (องศา)
    offsetBodyRoll  = sumBodyRoll / (float)GYRO_CAL_SAMPLES;
    offsetBodyPitch = sumBodyPitch / (float)GYRO_CAL_SAMPLES;

    // คำนวณค่าเฉลี่ย Offset มุมของ Platform (องศา)
    offsetPlatX = sumAngPX / (float)GYRO_CAL_SAMPLES;
    offsetPlatY = sumAngPY / (float)GYRO_CAL_SAMPLES;

    // รีเซ็ตค่าตัวแปรมุมให้เป็นศูนย์ทันที
    bodyRoll = 0; bodyPitch = 0; bodyYaw = 0;
    anglePlatformX = 0; anglePlatformY = 0;
    
    lastTime = millis();
    Serial.println("Calibration Done! All systems ZEROED.");
}

// --- [ ฟังก์ชันหลักสำหรับอ่านและคำนวณมุม ] ---
void Gyro() {
    sensors_event_t a, g, temp;
    
    // 1. คำนวณ Delta Time (dt)
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f; 
    if (dt <= 0) dt = 0.001f; 
    lastTime = currentTime;

    // --- [ 2. ดึงข้อมูล BODY (TCA 0) ] ---
    tcaSelect(0);
    mpuBody.getEvent(&a, &g, &temp);
    
    // คำนวณมุมจาก Accelerometer และหักลบ Offset
    float accRollBody  = (atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI) - offsetBodyRoll;
    float accPitchBody = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI) - offsetBodyPitch;

    // แปลง Gyro Rate จาก rad/s (หัก bias) เป็น deg/s
    float gyroX_deg = (g.gyro.x - biasBodyX) * 180.0f / PI;
    float gyroY_deg = (g.gyro.y - biasBodyY) * 180.0f / PI;
    float gyroZ_deg = (g.gyro.z - biasBodyZ) * 180.0f / PI;

    // Complementary Filter ผสม Accel (ระยะยาว) + Gyro (ระยะสั้น)
    bodyRoll  = COMP_FILTER_GAIN * (bodyRoll + gyroX_deg * dt) + (1.0f - COMP_FILTER_GAIN) * accRollBody;
    bodyPitch = COMP_FILTER_GAIN * (bodyPitch + gyroY_deg * dt) + (1.0f - COMP_FILTER_GAIN) * accPitchBody;
    
    // Yaw: สะสมค่า Gyro Z (ใช้ในการเลี้ยวใน EKF)
    bodyYaw += gyroZ_deg * dt; 

    // --- [ 3. ดึงข้อมูล PLATFORM (TCA 1) ] ---
    tcaSelect(1);
    mpuPlatform.getEvent(&a, &g, &temp);

    // คำนวณมุมดิบ (องศา) และหัก Offset
    float rawPlatX = (atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI) - offsetPlatX;
    float rawPlatY = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI) - offsetPlatY;

    // Low-pass Filter เพื่อให้ Platform นิ่งที่สุด
    anglePlatformX = (ALPHA_PLAT * rawPlatX) + ((1.0f - ALPHA_PLAT) * anglePlatformX);
    anglePlatformY = (ALPHA_PLAT * rawPlatY) + ((1.0f - ALPHA_PLAT) * anglePlatformY);

    // --- [ 4. เก็บข้อมูลลง msg_sensors เพื่อส่งไป ROS 2 ] ---
    // [0]=Roll, [1]=Pitch, [2]=Yaw (หัวใจหลักของ EKF)
    msg_sensors.data.data[0] = bodyRoll;       
    msg_sensors.data.data[1] = bodyPitch;      
    msg_sensors.data.data[2] = bodyYaw;        
    // ความเร็วเชิงมุม (deg/s) สำหรับ EKF angular_velocity
    msg_sensors.data.data[3] = gyroX_deg;      
    msg_sensors.data.data[4] = gyroY_deg;      
    msg_sensors.data.data[5] = gyroZ_deg;      
    // ข้อมูลสำหรับคุม Platform
    msg_sensors.data.data[6] = anglePlatformX; 
    msg_sensors.data.data[7] = anglePlatformY; 
}