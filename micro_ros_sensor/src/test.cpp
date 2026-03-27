// #include <Arduino.h>  // <--- ต้องมีบรรทัดนี้ใน PlatformIO
// #include <Wire.h>

// #define TCAADDR 0x70

// void tcaSelect(uint8_t i) {
//   if (i > 7) return;
//   Wire.beginTransmission(TCAADDR);
//   Wire.write(1 << i);
//   Wire.endTransmission();
// }

// void setup() {
//   // เริ่มต้น Serial
//   Serial.begin(115200);
//   while (!Serial); 

//   // เริ่มต้น I2C (SDA=21, SCL=22 สำหรับ ESP32)
//   Wire.begin(21, 22); 
  
//   Serial.println("\n--- TCA9548A I2C Scanner ---");
// }

// void loop() {
//   for (uint8_t t = 0; t < 8; t++) {
//     tcaSelect(t);
//     Serial.print("Scanning Channel ");
//     Serial.print(t);
//     Serial.print(": ");

//     bool found = false;
//     for (uint8_t addr = 1; addr < 127; addr++) {
//       if (addr == TCAADDR) continue; 

//       Wire.beginTransmission(addr);
//       uint8_t error = Wire.endTransmission(); // ใช้ uint8_t แทน byte เพื่อความชัวร์

//       if (error == 0) {
//         Serial.print("Found device at 0x");
//         if (addr < 16) Serial.print("0");
//         Serial.print(addr, HEX);
        
//         if (addr == 0x68 || addr == 0x69) Serial.print(" (MPU6050)");
//         else if (addr == 0x77 || addr == 0x76) Serial.print(" (MS5611)");
//         else if (addr == 0x36) Serial.print(" (AS5600)"); // แถมรหัส Magnetic Encoder ให้ด้วย
        
//         Serial.print(" | ");
//         found = true;
//       }
//     }
//     if (!found) Serial.print("No devices found.");
//     Serial.println();
//   }

//   Serial.println("--- Scan Finished ---");
//   delay(5000); 
// }