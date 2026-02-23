// #include <Arduino.h>
// #include <Wire.h>
// #include <AS5600.h>

// // micro-ROS libraries
// #include <micro_ros_arduino.h>
// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/float32.h>

// #define TCA_ADDR 0x70
// #define ENCODER_CH 5

// AS5600 as5600(&Wire);

// // micro-ROS entities
// rcl_publisher_t publisher;
// std_msgs__msg__Float32 msg;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;

// #define LED_PIN 2 // ไฟสถานะบน ESP32

// // Macro เช็ค Error ของ micro-ROS
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// void error_loop(){
//   while(1){
//     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//     delay(100);
//   }
// }

// void tcaSelect(uint8_t i) {
//   if (i > 7) return;
//   Wire.beginTransmission(TCA_ADDR);
//   Wire.write(1 << i);
//   Wire.endTransmission();
// }

// void setup() {
//   set_microros_transports(); // ตั้งค่าการเชื่อมต่อ (Serial)
  
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);  

//   Wire.begin(21, 22); 
//   Wire.setClock(400000);

//   // เตรียมความพร้อม AS5600
//   tcaSelect(ENCODER_CH);
//   as5600.begin();

//   // --- เริ่มต้น micro-ROS ---
//   allocator = rcl_get_default_allocator();

//   // สร้าง Support และ Node
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
//   RCCHECK(rclc_node_init_default(&node, "as5600_node", "", &support));

//   // สร้าง Publisher (Topic: Encoder)
//   RCCHECK(rclc_publisher_init_default(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//     "Encoder"));

//   msg.data = 0.0;
// }

// void loop() {
//   // เลือกช่อง I2C และอ่านค่า
//   tcaSelect(ENCODER_CH);
  
//   // แปลงค่าเป็นองศา (0 - 360)
//   float degrees = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
  
//   // ใส่ข้อมูลลงใน Message และ Publish
//   msg.data = degrees;
  
//   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

//   delay(50); // ปรับความถี่ตามต้องการ
// }