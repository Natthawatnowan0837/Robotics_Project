// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// // --- [ การตั้งค่า TCA9548A ] ---
// #define TCAADDR 0x70

// // --- [ การตั้งค่าตัวแปร IMU ] ---
// Adafruit_MPU6050 mpuBody;
// Adafruit_MPU6050 mpuPlatform;

// #define GYRO_CAL_SAMPLES 200 
// #define COMP_FILTER_GAIN 0.96f 

// // ตัวแปรเก็บค่ามุม
// float bodyRoll = 0, bodyPitch = 0, bodyYaw = 0;
// // ตัวแปรสำหรับ Bias ของ Gyro เท่านั้น (ไม่ Reset มุม Accel)
// float biasBodyX = 0, biasBodyY = 0, biasBodyZ = 0; 

// unsigned long lastTime = 0;

// // ฟังก์ชันเลือกช่อง TCA
// void tcaSelect(uint8_t i) {
//     if (i > 7) return;
//     Wire.beginTransmission(TCAADDR);
//     Wire.write(1 << i);
//     Wire.endTransmission();
// }

// // ฟังก์ชันสแกน I2C ในแต่ละช่อง
// void scanTCA() {
//     Serial.println("\n--- TCA9548A Scanner ---");
//     for (uint8_t t = 0; t < 2; t++) { // สแกนแค่ช่อง 0 และ 1 ที่เราใช้
//         tcaSelect(t);
//         Serial.print("Channel "); Serial.print(t);
//         bool found = false;
//         for (uint8_t addr = 1; addr <= 127; addr++) {
//             if (addr == TCAADDR) continue;
//             Wire.beginTransmission(addr);
//             if (Wire.endTransmission() == 0) {
//                 Serial.print(": Found 0x"); Serial.println(addr, HEX);
//                 found = true;
//             }
//         }
//         if (!found) Serial.println(": No device");
//     }
//     Serial.println("------------------------\n");
// }

// void calibrateGyroOnly() {
//     Serial.println(">>> Calibrating Gyro Bias... Keep Steady! <<<");
//     sensors_event_t a, g, temp;
//     float sumBX = 0, sumBY = 0, sumBZ = 0;

//     for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
//         tcaSelect(0);
//         mpuBody.getEvent(&a, &g, &temp);
//         sumBX += g.gyro.x; 
//         sumBY += g.gyro.y;
//         sumBZ += g.gyro.z;
//         delay(5);
//     }
//     biasBodyX = sumBX / (float)GYRO_CAL_SAMPLES;
//     biasBodyY = sumBY / (float)GYRO_CAL_SAMPLES;
//     biasBodyZ = sumBZ / (float)GYRO_CAL_SAMPLES;

//     // หมายเหตุ: ไม่มีการเก็บ Offset ของ Accel เพื่อให้ค่ามุมอิงตามแรงโน้มถ่วงโลกจริง
//     Serial.println("Gyro Bias Calibrated.");
// }

// void setup() {
//     Serial.begin(115200);
//     Wire.begin(21, 22); // SDA=21, SCL=22
    
//     scanTCA(); // สแกนก่อนเริ่ม

//     // เริ่มต้น MPU6050 ในช่องที่ 0
//     tcaSelect(0);
//     if (!mpuBody.begin()) {
//         Serial.println("Failed to find MPU6050 on Channel 0!");
//     }

//     calibrateGyroOnly();
//     lastTime = millis();
// }

// void loop() {
//     sensors_event_t a, g, temp;
//     unsigned long currentTime = millis();
//     float dt = (currentTime - lastTime) / 1000.0f; 
//     if (dt <= 0) dt = 0.001f; 
//     lastTime = currentTime;

//     tcaSelect(0);
//     mpuBody.getEvent(&a, &g, &temp);
    
//     // 1. คำนวณมุมจาก Accel (อิงตามโลกจริง ไม่หัก Offset)
//     float accRollBodyRad  = atan2(a.acceleration.y, a.acceleration.z);
//     float accPitchBodyRad = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));

//     // 2. ค่า Gyro หัก Bias
//     float gyroX_rads = (g.gyro.x - biasBodyX);
//     float gyroY_rads = (g.gyro.y - biasBodyY);
//     float gyroZ_rads = (g.gyro.z - biasBodyZ);

//     // 3. Complementary Filter
//     bodyRoll  = COMP_FILTER_GAIN * (bodyRoll + gyroX_rads * dt) + (1.0f - COMP_FILTER_GAIN) * accRollBodyRad;
//     bodyPitch = COMP_FILTER_GAIN * (bodyPitch + gyroY_rads * dt) + (1.0f - COMP_FILTER_GAIN) * accPitchBodyRad;
//     bodyYaw  += gyroZ_rads * dt; 

//     // พิมพ์ค่าออกทาง Serial เพื่อดู Real-time
//     Serial.print("Roll:"); Serial.print(bodyRoll * 180/PI); Serial.print(",");
//     Serial.print("Pitch:"); Serial.print(bodyPitch * 180/PI); Serial.print(",");
//     Serial.print("Yaw:"); Serial.println(bodyYaw * 180/PI);

//     delay(10); // ปรับตามความเร็วที่ต้องการ
// }