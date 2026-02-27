// #include <Arduino.h>
// #include <Wire.h>
// #include <MPU6050_light.h>

// #define TCA_ADDR 0x70

// MPU6050 mpu(Wire);

// // ฟังก์ชันเลือกช่องบน TCA9548A
// void tcaSelect(uint8_t i) {
//   if (i > 7) return;
//   Wire.beginTransmission(TCA_ADDR);
//   Wire.write(1 << i);
//   Wire.endTransmission();
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(21, 22); // SDA=21, SCL=22
//   Wire.setClock(400000);

//   Serial.println("\n--- TCA9548A Channel 0 Test ---");

//   // 1. เลือกช่อง 0
//   tcaSelect(0);
//   delay(100);

//   // 2. เริ่มต้น MPU6050 ในช่อง 0
//   byte status = mpu.begin();
//   if (status != 0) {
//     Serial.print("MPU6050 Ch 0 Error: ");
//     Serial.println(status);
//     Serial.println("Check: 1. Is sensor on SD0/SC0? 2. Is VCC connected?");
//     while (1); // หยุดถ้าเชื่อมต่อไม่สำเร็จ
//   }

//   Serial.println("Connection Successful!");
  
//   // 3. คำนวณ Offset (วางเซนเซอร์ไว้นิ่งๆ)
//   Serial.println("Calculating offsets... Don't move the sensor.");
//   delay(1000);
//   mpu.calcOffsets(true, true);
//   Serial.println("Offsets calculated! Ready to read.");
// }

// void loop() {
//   // ต้องเลือกช่อง 0 เสมอก่อนอ่าน
//   tcaSelect(0);
//   mpu.update();

//   // ปริ้นค่าออกทาง Serial Monitor
//   Serial.print("Angle X: "); Serial.print(mpu.getAngleX());
//   Serial.print("\tAngle Y: "); Serial.print(mpu.getAngleY());
//   Serial.print("\tAngle Z: "); Serial.println(mpu.getAngleZ());

//   delay(100); // อัปเดตทุก 0.1 วินาที
// } 