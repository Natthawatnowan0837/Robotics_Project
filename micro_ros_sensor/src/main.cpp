#include "main.h"

// --- Micro-ROS objects ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;
rcl_publisher_t pub_motor , pub_sensors;
std_msgs__msg__Float32MultiArray msg_motor , msg_sensors; 

MS5611 ms5611(0x77);

float sensors[11];
float motor_data[8];

void tcaSelect(uint8_t i) {
    if (i > 7) return;
    Wire.beginTransmission(0x70); 
    Wire.write(1 << i);
    Wire.endTransmission();
}


void error_loop() {
    while(1) {
        Serial.println("micro-ROS Error!");
        delay(100);
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    if (timer != NULL) {
        // 1. อ่านค่า Sensor ทั้งหมดก่อนส่ง
        Encoder_motor();
        Encoder_arm();
        Gyro();
        hall_effect();
        pressure();

        // 2. Publish ข้อมูลที่อ่านได้
        RCSOFTCHECK(rcl_publish(&pub_motor, &msg_motor, NULL));
        RCSOFTCHECK(rcl_publish(&pub_sensors, &msg_sensors, NULL));
    }
}

float lowpassFilter(float input, float prev_output, float alpha) {
    return (alpha * input) + ((1.0f - alpha) * prev_output);
}

float medianFilter(float* data, int size) { 
    float sorted[size]; 
    memcpy(sorted, data, size * sizeof(float)); 
    for (int i = 0; i < size-1; i++) { 
        for (int j = 0; j < size-i-1; j++) { 
            if (sorted[j] > sorted[j+1]) { 
                float temp = sorted[j]; 
                sorted[j] = sorted[j+1]; 
                sorted[j+1] = temp; 
            } 
        } 
    } 
    return sorted[size/2]; 
}


void setup() {
    Serial.begin(115200);
    set_microros_transports();
    pinMode(hallPin, INPUT); // เพิ่มบรรทัดนี้เพื่อตั้งค่า Transport (Serial)
    Wire.begin(21, 22);
    Wire.setClock(400000);

    tcaSelect(0); mpuBody.begin();
    delay(10);
    tcaSelect(1); mpuPlatform.begin();
    delay(10);
    tcaSelect(2); as5600_motor.begin();
    delay(10);
    tcaSelect(3); as5600_motor.begin();
    delay(10);
    tcaSelect(4); as5600_arm.begin();
    delay(10);
    tcaSelect(5); as5600_arm.begin();
    delay(10);
    tcaSelect(6); ms5611.begin();
    delay(10);
    // tcaSelect(6); pressure.begin();
    calibrateSensors();
    allocator = rcl_get_default_allocator();

    // --- การจอง Memory สำหรับ MultiArray (สำคัญมาก) ---
    msg_motor.data.capacity = 8;
    msg_motor.data.size = 8;
    msg_motor.data.data = motor_data;

    msg_motor.layout.dim.capacity = 0;
    msg_motor.layout.dim.size = 0;
    msg_motor.layout.data_offset = 0;
    // -----------------------------------------------
    msg_sensors.data.capacity = 11;
    msg_sensors.data.size = 11;
    msg_sensors.data.data = sensors;
    
    msg_sensors.layout.dim.capacity = 0;
    msg_sensors.layout.dim.size = 0;
    msg_sensors.layout.data_offset = 0;

if (ms5611.begin()) {
        // หัวใจสำคัญ: เลข 1 ใน reset(1) คือการเปิด Adjusted Math
        ms5611.reset(1); 
        // ตั้งค่าความละเอียดสูงสุดเพื่อความแม่นยำ
        ms5611.setOversampling(OSR_ULTRA_HIGH); 
        Serial.println("MS5611 Initialized with Adjusted Math.");
    } else {
        Serial.println("MS5611 NOT found on Channel 6!");
    }

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_sensor_node", "", &support));

    // Publishers
    RCCHECK(rclc_publisher_init_default(&pub_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motors"));
    RCCHECK(rclc_publisher_init_default(&pub_sensors, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "sensors"));
    
    // Timer & Executor
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // เลข 1 คือจำนวน timer
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    Serial.println("Encoder & micro-ROS Initialized");
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}