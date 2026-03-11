#include "main.h" // ตรวจสอบว่ามี Library Adafruit_MPU6050, Adafruit_Sensor และ Wire

// --- [ การตั้งค่าตัวแปร IMU ] ---
Adafruit_MPU6050 mpuBody;
Adafruit_MPU6050 mpuPlatform;

#define GYRO_CAL_SAMPLES 200 
#define ALPHA_PLAT 0.1f       // Low-pass Filter สำหรับ Platform (0.1 = นิ่งมากแต่ช้า)
#define COMP_FILTER_GAIN 0.96f // Complementary Filter (0.96 เชื่อ Gyro, 0.04 เชื่อ Accel)

// --- [ ตัวแปรเก็บค่ามุม (Filtered & Integration) ] ---
float bodyRoll = 0, bodyPitch = 0, bodyYaw = 0;
float anglePlatformX = 0, anglePlatformY = 0;

// --- [ ตัวแปรสำหรับ Calibration (Offset/Bias) ] ---
float biasBodyX = 0, biasBodyY = 0, biasBodyZ = 0; 
float offsetPlatX = 0, offsetPlatY = 0;

unsigned long lastTime = 0;


// --- [ ฟังก์ชัน Set ศูนย์ (Calibration) ] ---
void calibrateSensors() {
    Serial.println("Calibrating IMUs... Keep it STEADY!");
    sensors_event_t a, g, temp;
    
    float sumBX = 0, sumBY = 0, sumBZ = 0;
    float sumAngPX = 0, sumAngPY = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        // 1. อ่านค่าจาก Body (TCA 0) หา Bias Gyro (rad/s)
        tcaSelect(0);
        mpuBody.getEvent(&a, &g, &temp);
        sumBX += g.gyro.x; 
        sumBY += g.gyro.y;
        sumBZ += g.gyro.z;

        // 2. อ่านค่าจาก Platform (TCA 1) หา Offset มุมเริ่มต้นจากแรงโน้มถ่วง
        tcaSelect(1);
        mpuPlatform.getEvent(&a, &g, &temp);
        sumAngPX += atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
        sumAngPY += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
        
        delay(5);
    }

    // คำนวณค่าเฉลี่ย Bias ของ Gyro Body
    biasBodyX = sumBX / (float)GYRO_CAL_SAMPLES;
    biasBodyY = sumBY / (float)GYRO_CAL_SAMPLES;
    biasBodyZ = sumBZ / (float)GYRO_CAL_SAMPLES;

    // คำนวณค่าเฉลี่ย Offset ของมุม Platform
    offsetPlatX = sumAngPX / (float)GYRO_CAL_SAMPLES;
    offsetPlatY = sumAngPY / (float)GYRO_CAL_SAMPLES;

    // รีเซ็ตค่าเริ่มต้น
    bodyRoll = 0; bodyPitch = 0; bodyYaw = 0;
    anglePlatformX = 0; anglePlatformY = 0;
    
    lastTime = millis();
    Serial.println("Calibration Done. System Zeroed.");
}

// --- [ ฟังก์ชันหลักสำหรับอ่านและคำนวณมุม ] ---
void Gyro() {
    sensors_event_t a, g, temp;
    
    // 1. คำนวณ Delta Time (dt) เพื่อใช้ในการ Integrate มุม
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f; // แปลงเป็นวินาที
    if (dt <= 0) dt = 0.001f; // ป้องกันการหารศูนย์
    lastTime = currentTime;

    // --- [ 2. ดึงข้อมูล BODY (TCA 0) ] ---
    tcaSelect(0);
    mpuBody.getEvent(&a, &g, &temp);
    
    // คำนวณมุมจาก Accelerometer (Static Tilt) - องศา
    float accRollBody  = atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
    float accPitchBody = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;

    // แปลง Gyro Rate จาก rad/s (หัก bias) เป็น deg/s
    float gyroX_deg = (g.gyro.x - biasBodyX) * 180.0f / PI;
    float gyroY_deg = (g.gyro.y - biasBodyY) * 180.0f / PI;
    float gyroZ_deg = (g.gyro.z - biasBodyZ) * 180.0f / PI;

    // Complementary Filter: ผสมความนิ่งของ Accel กับความเร็วของ Gyro
    bodyRoll  = COMP_FILTER_GAIN * (bodyRoll + gyroX_deg * dt) + (1.0f - COMP_FILTER_GAIN) * accRollBody;
    bodyPitch = COMP_FILTER_GAIN * (bodyPitch + gyroY_deg * dt) + (1.0f - COMP_FILTER_GAIN) * accPitchBody;
    
    // Yaw: ได้จากการสะสมค่า Gyro Z เท่านั้น (มุมจะไหลเล็กน้อยตามคุณภาพ Sensor)
    bodyYaw += gyroZ_deg * dt; 

    // --- [ 3. ดึงข้อมูล PLATFORM (TCA 1) ] ---
    tcaSelect(1);
    mpuPlatform.getEvent(&a, &g, &temp);

    // คำนวณมุมดิบ (องศา) และหัก Offset
    float rawPlatX = (atan2(a.acceleration.y, a.acceleration.z) * 180.0f / PI) - offsetPlatX;
    float rawPlatY = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI) - offsetPlatY;

    // Low-pass Filter สำหรับ Platform (เน้นความนิ่ง)
    anglePlatformX = (ALPHA_PLAT * rawPlatX) + ((1.0f - ALPHA_PLAT) * anglePlatformX);
    anglePlatformY = (ALPHA_PLAT * rawPlatY) + ((1.0f - ALPHA_PLAT) * anglePlatformY);

    // --- [ 4. เก็บข้อมูลลง msg_sensors (หน่วยองศา) ] ---
    msg_sensors.data.data[0] = bodyRoll;       // Roll Body (deg)
    msg_sensors.data.data[1] = bodyPitch;      // Pitch Body (deg)
    msg_sensors.data.data[2] = bodyYaw;        // Yaw Body (deg)
    msg_sensors.data.data[3] = gyroX_deg;      // Rate X (deg/s)
    msg_sensors.data.data[4] = gyroY_deg;      // Rate Y (deg/s)
    msg_sensors.data.data[5] = gyroZ_deg;      // Rate Z (deg/s)
    msg_sensors.data.data[6] = anglePlatformX; // Roll Platform (deg)
    msg_sensors.data.data[7] = anglePlatformY; // Pitch Platform (deg)
}