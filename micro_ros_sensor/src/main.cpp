#include "main.h"

// --- Micro-ROS objects ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;
rcl_publisher_t pub_motor_rps, pub_arm_degrees,
                pub_imu_body, pub_imu_platform,
                pub_hall_effect;

std_msgs__msg__Bool msg_hall_effect;
geometry_msgs__msg__Vector3 msg_imu_body, msg_imu_platform;
std_msgs__msg__Float32MultiArray msg_motor_rps  ,msg_arm_degrees; 


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

        // 2. Publish ข้อมูลที่อ่านได้
        RCSOFTCHECK(rcl_publish(&pub_motor_rps, &msg_motor_rps, NULL));
        RCSOFTCHECK(rcl_publish(&pub_arm_degrees, &msg_arm_degrees, NULL));
        RCSOFTCHECK(rcl_publish(&pub_imu_body, &msg_imu_body, NULL));
        RCSOFTCHECK(rcl_publish(&pub_imu_platform, &msg_imu_platform, NULL));
        RCSOFTCHECK(rcl_publish(&pub_hall_effect, &msg_hall_effect, NULL));
    }
}
float motor_data[2] = {0.0f, 0.0f};
float arm_data[2] = {0.0f, 0.0f};
bool imu_online[2] = {false, false};

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
    // tcaSelect(6); pressure.begin();

    allocator = rcl_get_default_allocator();

    // --- การจอง Memory สำหรับ MultiArray (สำคัญมาก) ---
    msg_motor_rps.data.capacity = 2;
    msg_motor_rps.data.size = 2;
    msg_motor_rps.data.data = motor_data;

    msg_arm_degrees.data.capacity = 2;
    msg_arm_degrees.data.size = 2;
    msg_arm_degrees.data.data = arm_data;
    // -----------------------------------------------
    msg_motor_rps.layout.dim.capacity = 0;
    msg_motor_rps.layout.dim.size = 0;
    msg_motor_rps.layout.data_offset = 0;

    msg_arm_degrees.layout.dim.capacity = 0;
    msg_arm_degrees.layout.dim.size = 0;
    msg_arm_degrees.layout.data_offset = 0;
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_sensor_node", "", &support));

    // Publishers
    RCCHECK(rclc_publisher_init_default(&pub_motor_rps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motor_rps_array"));
    RCCHECK(rclc_publisher_init_default(&pub_arm_degrees, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "arm_deg_array"));
    RCCHECK(rclc_publisher_init_default(&pub_imu_body, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "gyro_body"));
    RCCHECK(rclc_publisher_init_default(&pub_imu_platform, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "gyro_plateform"));
    RCCHECK(rclc_publisher_init_default(&pub_hall_effect, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "hall_effect"));
    // Timer & Executor
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // เลข 1 คือจำนวน timer
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    Serial.println("Encoder & micro-ROS Initialized");
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}