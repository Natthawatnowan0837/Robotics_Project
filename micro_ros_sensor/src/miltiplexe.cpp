// #include <Wire.h>
// #include <Arduino.h>

// #define TCAADDR 0x70

// // ฟังก์ชันสำหรับเลือก Channel
// void tcaselect(uint8_t i) {
//   if (i > 7) return;
//   Wire.beginTransmission(TCAADDR);
//   Wire.write(1 << i);
//   Wire.endTransmission();
// }

// // ฟังก์ชันปิดทุก Channel
// void tcaoff() {
//   Wire.beginTransmission(TCAADDR);
//   Wire.write(0); 
//   Wire.endTransmission();
// }

// void setup() {
//   Serial.begin(115200);
//   while (!Serial); 
  
//   Wire.begin(21, 22); 
//   Wire.setClock(100000); 
  
//   Serial.println("\n*********************************");
//   Serial.println("TCA9548A Connection & Bus Scanner");
//   Serial.println("*********************************");

//   // --- ส่วนที่เพิ่ม: เช็คว่าเจอตัว TCA9548A หรือไม่ ---
//   Wire.beginTransmission(TCAADDR);
//   byte error = Wire.endTransmission();

//   if (error == 0) {
//     Serial.println("[SUCCESS] TCA9548A found at Address 0x70");
//   } else {
//     Serial.println("[ERROR] TCA9548A NOT FOUND!");
//     Serial.println("Please check:");
//     Serial.println("1. SDA (GPIO 21) & SCL (GPIO 22) connections");
//     Serial.println("2. VCC (3.3V) & GND power supply to TCA");
//     Serial.println("3. TCA Address pins (A0, A1, A2) should be GND for 0x70");
//     while (1); // หยุดการทำงานถ้าไม่เจอตัวหลัก
//   }
//   // -------------------------------------------
// }

// void loop() {
//   for (uint8_t t = 0; t < 8; t++) {
//     tcaoff();      
//     delay(10);
//     tcaselect(t);  
    
//     Serial.print("Channel "); 
//     Serial.print(t); 
//     Serial.print(": ");
    
//     int devicesCount = 0;

//     for (uint8_t addr = 1; addr <= 127; addr++) {
//       if (addr == TCAADDR) continue;

//       Wire.beginTransmission(addr);
//       byte error = Wire.endTransmission();

//       if (error == 0) {
//         Serial.printf("[0x%02X] ", addr);
//         devicesCount++;
//       }
//     }

//     if (devicesCount == 0) Serial.print("---");
//     Serial.println();
//     delay(20); 
//   }

//   Serial.println("---------- Scan Finished ----------");
//   delay(5000); 
// }