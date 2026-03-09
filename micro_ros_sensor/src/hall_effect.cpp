#include "main.h"
const int hallPin = 5;     // ขาที่ต่อกับ Out ของเซนเซอร์
int hallState = 0;         // ตัวแปรเก็บสถานะ


void hall_effect() {
  hallState = digitalRead(hallPin);

  if (hallState == LOW) {  
    msg_sensors.data.data[8] = 1; // ส่งข้อมูลว่าเจอแม่เหล็ก
  } else {
    msg_sensors.data.data[8] = 0; // ส่งข้อมูลว่าไม่เจอแม่เหล็ก
  }
}