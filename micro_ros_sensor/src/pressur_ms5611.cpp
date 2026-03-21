#include "main.h"

// บอกคอมไพเลอร์ว่า ms5611 และ msg_sensors ถูกประกาศไว้ที่ main.cpp
extern MS5611 ms5611; 
extern std_msgs__msg__Float32MultiArray msg_sensors; // สมมติว่าเป็น Type นี้ตามมาตรฐาน micro-ROS

SimpleKalmanFilter pressureKalman(0.1, 0.1, 0.05);
SimpleKalmanFilter tempKalman(0.1, 0.1, 0.05);

float current_pressure = 0.0;
float current_temperature = -999.0;
unsigned long last_read_time = 0;

void pressure() {
    tcaSelect(6); // เลือกช่อง 6 บน I2C Multiplexer
    
    static bool initialized = false;
    if (!initialized) {
        if (ms5611.begin()) {
            ms5611.setOversampling(OSR_ULTRA_HIGH);
            initialized = true;
        } else {
            return; // ถ้า Initialize ไม่สำเร็จ ให้ข้ามไปก่อน
        }
    }

    // อ่านค่าทุก 500ms
    if (millis() - last_read_time >= 500) {
        last_read_time = millis();
        
        int result = ms5611.read();
        if (result == MS5611_READ_OK) {
            // 1. อัปเดตค่าเข้า Kalman Filter
            current_pressure = pressureKalman.updateEstimate((float)ms5611.getPressure());
            current_temperature = tempKalman.updateEstimate((float)ms5611.getTemperature());

            // 2. ใส่ค่าลงในโครงสร้างข้อมูลของ micro-ROS
            // index [9] = Pressure, index [10] = Temperature
            msg_sensors.data.data[9] = current_pressure;
            msg_sensors.data.data[10] = current_temperature;
        }
    }
}