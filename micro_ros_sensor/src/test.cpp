// #include <Arduino.h>
// #include <Wire.h>
// #include <MS5611.h>

// #define TCAADDR 0x70
// #define MS5611_CHANNEL 6

// // ==========================================
// // 1. นิยาม Class SimpleKalmanFilter (ย้ายออกมาไว้นอก loop)
// // ==========================================
// class SimpleKalmanFilter {
//   private:
//     float err_measure;
//     float err_estimate;
//     float q;
//     float current_estimate;
//     float last_estimate;
//     float kalman_gain;
//     bool is_initialized;

//   public:
//     SimpleKalmanFilter(float mea_e, float est_e, float q) {
//       err_measure = mea_e;
//       err_estimate = est_e;
//       this->q = q;
//       is_initialized = false;
//     }

//     float updateEstimate(float mea) {
//       if (!is_initialized) {
//         last_estimate = mea;
//         current_estimate = mea;
//         is_initialized = true;
//       }
//       kalman_gain = err_estimate / (err_estimate + err_measure);
//       current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
//       err_estimate =  (1.0 - kalman_gain) * err_estimate + fabs(last_estimate - current_estimate) * q;
//       last_estimate = current_estimate;
//       return current_estimate;
//     }
// };

// // ==========================================
// // 2. ประกาศ Object และตัวแปร Global
// // ==========================================
// MS5611 ms5611(0x77); 

// // สร้าง Object ของ Kalman Filter (ตั้งค่า Tuning ตามที่คุณให้มาล่าสุด)
// SimpleKalmanFilter pressureKalman(0.1, 0.1, 0.05);
// SimpleKalmanFilter tempKalman(0.1, 0.1, 0.05);

// unsigned long last_read_time = 0;
// const long interval = 500; 

// // ฟังก์ชันเลือกช่อง TCA
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
//     Wire.setClock(100000); 

//     Serial.println("\n--- MS5611 with Custom Kalman Class ---");

//     tcaSelect(MS5611_CHANNEL);
//     delay(10);

//     if (ms5611.begin() == true) {
//         Serial.print("✅ MS5611 found at Channel ");
//         Serial.println(MS5611_CHANNEL);
//     } else {
//         Serial.println("❌ MS5611 NOT found!");
//         while (1);
//     }

//     ms5611.reset(1); 
//     ms5611.setOversampling(OSR_ULTRA_HIGH);
// }

// void loop() {
//     // อ่านค่าทุกๆ 500ms โดยไม่ใช้ delay
//     if (millis() - last_read_time >= interval) {
//         last_read_time = millis();

//         tcaSelect(MS5611_CHANNEL);

//         int result = ms5611.read();

//         if (result == MS5611_READ_OK) {
//             float raw_p = ms5611.getPressure();
//             float raw_t = ms5611.getTemperature();

//             // ใช้งาน Kalman Filter ที่สร้างจาก Class ด้านบน
//             float filtered_p = pressureKalman.updateEstimate(raw_p);
//             float filtered_t = tempKalman.updateEstimate(raw_t);

//             // แสดงผลเทียบกันให้เห็นความแตกต่าง
//             Serial.print("Pressure Filtered: "); Serial.print(filtered_p, 2);
//             Serial.print(" | Temp Filtered: "); Serial.println(filtered_t, 2);
//         } else {
//             Serial.println("⚠️ Sensor Read Error!");
//         }
//     }
// }