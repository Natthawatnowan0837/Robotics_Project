// #include <Arduino.h>
// #include <Wire.h>
// #include <AS5600.h>
// #define TCAADDR 0x70
// #define MEDIAN_SIZE 5
// #define RAW_TO_DEG ((360.0 / 4096.0) * -1.0)

// AS5600 as5600_motor(&Wire);

// // ===== ตัวแปร =====
// unsigned long last_time_motor = 0;
// float last_angle_l = 0, last_angle_r = 0;
// float buffer_l[MEDIAN_SIZE] = {0};
// float buffer_r[MEDIAN_SIZE] = {0};
// float filtered_rps_l = 0, filtered_rps_r = 0;

// // ===== เลือก Channel =====
// void tcaselect(uint8_t i) {
//   if (i > 7) return;
//   Wire.beginTransmission(TCAADDR);
//   Wire.write(1 << i);
//   Wire.endTransmission();
// }

// // ===== Median Filter =====
// float getMedian(float* data, int size) {
//   float sorted[MEDIAN_SIZE];
//   memcpy(sorted, data, size * sizeof(float));

//   for (int i = 0; i < size - 1; i++) {
//     for (int j = 0; j < size - i - 1; j++) {
//       if (sorted[j] > sorted[j + 1]) {
//         float temp = sorted[j];
//         sorted[j] = sorted[j + 1];
//         sorted[j + 1] = temp;
//       }
//     }
//   }
//   return sorted[size / 2];
// }

// // ===== อ่าน Encoder =====
// void Encoder_motor() {

//   unsigned long current_time = micros();
//   float dt = (current_time - last_time_motor) / 1000000.0;

//   if (dt < 0.005) return;

//   const float alpha = 0.5;

//   // ---------- ล้อซ้าย (Ch 2) ----------
//   tcaselect(2);
//   float current_angle_l = as5600_motor.readAngle() * RAW_TO_DEG;
//   float delta_l = current_angle_l - last_angle_l;

//   if (delta_l > 180) delta_l -= 360;
//   else if (delta_l < -180) delta_l += 360;

//   float raw_rps_l = (delta_l / 360.0) / dt;

//   for (int i = MEDIAN_SIZE - 1; i > 0; i--)
//     buffer_l[i] = buffer_l[i - 1];
//   buffer_l[0] = raw_rps_l;

//   filtered_rps_l = alpha * getMedian(buffer_l, MEDIAN_SIZE)
//                  + (1 - alpha) * filtered_rps_l;

//   // ---------- ล้อขวา (Ch 3) ----------
//   tcaselect(3);
//   float current_angle_r = as5600_motor.readAngle() * RAW_TO_DEG;
//   float delta_r = current_angle_r - last_angle_r;

//   if (delta_r > 180) delta_r -= 360;
//   else if (delta_r < -180) delta_r += 360;

//   float raw_rps_r = (delta_r / 360.0) / dt;

//   for (int i = MEDIAN_SIZE - 1; i > 0; i--)
//     buffer_r[i] = buffer_r[i - 1];
//   buffer_r[0] = raw_rps_r;

//   filtered_rps_r = alpha * getMedian(buffer_r, MEDIAN_SIZE)
//                  + (1 - alpha) * filtered_rps_r;

//   // ---------- Deadzone + ปัดทศนิยม ----------
//   float out_l = (abs(filtered_rps_l) < 0.02) ? 0.0f :
//                 roundf(filtered_rps_l * 100.0f) / 100.0f;

//   float out_r = (abs(filtered_rps_r) < 0.02) ? 0.0f :
//                 roundf(filtered_rps_r * 100.0f) / 100.0f;

//   // ---------- แสดงผล ----------
//   Serial.print("Left RPS: ");
//   Serial.print(out_l);
//   Serial.print(" | Right RPS: ");
//   Serial.println(out_r);

//   last_angle_l = current_angle_l;
//   last_angle_r = current_angle_r;
//   last_time_motor = current_time;
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(21, 22);
// }

// void loop() {
//   Encoder_motor();
// }