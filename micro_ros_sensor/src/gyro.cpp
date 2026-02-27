// #include "main.h"

// // ประกาศ Object ให้ตรงตาม Library MPU6050_light
// MPU6050 mpuBody(Wire);      // IMU ตัวที่ 1 (Body)
// MPU6050 mpuPlatform(Wire);  // IMU ตัวที่ 2 (Platform)

// void Gyro() {
//     // อ่าน IMU Body (Channel 0)
//     tcaSelect(0);
//     mpuBody.update();
//     msg_imu_body.x = mpuBody.getAngleX();
//     msg_imu_body.y = mpuBody.getAngleY();
//     msg_imu_body.z = mpuBody.getAngleZ();

//     // อ่าน IMU Platform (Channel 1)
//     tcaSelect(1);
//     mpuPlatform.update();
//     msg_imu_platform.x = mpuPlatform.getAngleX();
//     msg_imu_platform.y = mpuPlatform.getAngleY();
//     msg_imu_platform.z = mpuPlatform.getAngleZ();
// }