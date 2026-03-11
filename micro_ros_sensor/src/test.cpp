// #include <Arduino.h>
// #include <Wire.h>
// #include "MS5611.h"

// #define TCAADDR 0x70
// #define MS5611_CHANNEL 6

// // สร้าง Object สำหรับ MS5611 (ระบุ Address 0x77 หรือ 0x76 ตามโมดูลของคุณ)
// MS5611 ms5611(0x77); 

// // ฟังก์ชันเลือกช่อง (Channel) ของ TCA
// void tcaSelect(uint8_t i) {
//     if (i > 7) return;
//     Wire.beginTransmission(TCAADDR);
//     Wire.write(1 << i);
//     Wire.endTransmission();
// }

// void setup() {
//     Serial.begin(115200);
//     while (!Serial);
    
//     Wire.begin(21, 22);
//     Wire.setClock(100000); // 100kHz เพื่อความเสถียร

//     Serial.println("\n--- MS5611 with TCA9548A (Channel 6) ---");

//     // 1. เลือกช่อง 6 ก่อนเริ่มจัดการเซนเซอร์
//     tcaSelect(MS5611_CHANNEL);
//     delay(10);

//     // 2. เริ่มต้น MS5611
//     if (ms5611.begin() == true) {
//         Serial.print("MS5611 found at Channel ");
//         Serial.println(MS5611_CHANNEL);
//     } else {
//         Serial.println("MS5611 NOT found. Check wiring on Channel 6!");
//         while (1);
//     }

//     // 3. เปิดโหมด Adjusted Math (โหมดพิเศษของ Rob Tillaart)
//     // การใส่พารามิเตอร์ 1 ใน reset() จะเปิดใช้งานสูตรคำนวณที่แม่นยำขึ้น
//     ms5611.reset(1); 
    
//     // ตั้งค่าความละเอียดสูงสุด (Ultra High Oversampling)
//     ms5611.setOversampling(OSR_ULTRA_HIGH);
// }

// void loop() {
//     // เลือกช่อง 6 ทุกครั้งก่อนอ่านค่า (กรณีมีเซนเซอร์ช่องอื่นด้วย)
//     tcaSelect(MS5611_CHANNEL);

//     // อ่านค่าจากเซนเซอร์
//     int result = ms5611.read();

//     if (result == MS5611_READ_OK) {
//         Serial.print("Temperature: ");
//         Serial.print(ms5611.getTemperature(), 2);
//         Serial.print(" °C\t");
        
//         Serial.print("Pressure: ");
//         Serial.print(ms5611.getPressure(), 2);
//         Serial.println(" hPa");
//     } else {
//         Serial.print("Error in read: ");
//         Serial.println(result);
//     }

//     delay(1000); // รอ 1 วินาทีก่อนอ่านค่าถัดไป
// }