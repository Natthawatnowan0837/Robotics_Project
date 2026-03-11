#include "main.h"

// ลบหรือคอมเมนต์บรรทัด MS5611 ms5611; ออกจากไฟล์นี้!!
// เพราะเราจะไปประกาศตัวจริงไว้ใน main.cpp แทน

void pressure() {
    tcaSelect(6); // เลือกช่อง 6
    
    static bool initialized = false;
    if (!initialized) {
        if (ms5611.begin()) {
            ms5611.reset(1); // Adjusted Math
            ms5611.setOversampling(OSR_ULTRA_HIGH);
            initialized = true;
        }
    }

    if (ms5611.read() == MS5611_READ_OK) {
        msg_sensors.data.data[9] = ms5611.getPressure();
    }
}