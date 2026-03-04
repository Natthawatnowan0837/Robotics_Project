// #include <Arduino.h>
// #include <Wire.h>

// #define TCAADDR 0x70

// // ฟังก์ชันเลือกช่อง (Channel) ของ TCA
// void tcaSelect(uint8_t i) {
//     if (i > 7) return;
//     Wire.beginTransmission(TCAADDR);
//     Wire.write(1 << i);
//     if (Wire.endTransmission() != 0) {
//         Serial.printf("Error: Cannot communicate with TCA at Channel %d\n", i);
//     }
// }

// // ฟังก์ชันปิดทุกช่อง เพื่อล้างสถานะ Bus
// void tcaOff() {
//     Wire.beginTransmission(TCAADDR);
//     Wire.write(0);
//     Wire.endTransmission();
// }

// void setup() {
//     Serial.begin(115200);
//     delay(2000);
    
//     // เริ่มต้น I2C ที่ขา 21, 22
//     Wire.begin(21, 22);
//     Wire.setClock(100000); // ใช้ความเร็วต่ำ 100kHz เพื่อความเสถียร

//     Serial.println("\n========================================");
//     Serial.println("   TCA9548A & I2C DEVICE SCANNER");
//     Serial.println("========================================");

//     // --- ส่วนเช็คว่าเจอตัว TCA9548A หลักไหม ---
//     Wire.beginTransmission(TCAADDR);
//     if (Wire.endTransmission() == 0) {
//         Serial.println("[  OK  ] TCA9548A found at Address 0x70");
//     } else {
//         Serial.println("[FAILED] TCA9548A NOT FOUND!");
//         Serial.println(">>> Check: SDA(21), SCL(22), Power(3.3V), and Reset Pin (RST to 3.3V)");
//         while(1) delay(1000); // ถ้าไม่เจอตัวหลัก ให้หยุดรอเช็คสาย
//     }
// }

// void loop() {
//     Serial.println("\n--- Scanning all 8 Channels ---");

//     for (uint8_t chan = 0; chan < 8; chan++) {
//         tcaOff();      // ปิดช่องก่อนหน้า
//         delay(5);      // ให้เวลา Bus เคลียร์สัญญาณ
//         tcaSelect(chan); // เปิดช่องที่จะสแกน
        
//         Serial.print("Channel "); Serial.print(chan); Serial.print(": ");
        
//         int devicesFound = 0;
//         for (uint8_t addr = 1; addr < 127; addr++) {
//             if (addr == TCAADDR) continue; // ข้าม Address ของตัว TCA เอง

//             Wire.beginTransmission(addr);
//             if (Wire.endTransmission() == 0) {
//                 Serial.printf("[0x%02X] ", addr);
//                 devicesFound++;
//             }
//         }

//         if (devicesFound == 0) Serial.print("No devices");
//         Serial.println();
//     }

//     Serial.println("----------------------------------------");
//     Serial.println("Scanning again in 5 seconds...");
//     delay(5000); 
// }