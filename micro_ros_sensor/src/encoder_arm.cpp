#include "main.h"
AS5600 as5600_arm(&Wire);
void Encoder_arm() {
    const float RAW_TO_DEG = (360.0 / 4096.0);
    tcaSelect(4);
    float deg_l = as5600_arm.readAngle() * RAW_TO_DEG;
    tcaSelect(5);
    float deg_r = as5600_arm.readAngle() * RAW_TO_DEG;
    
    msg_arm_degrees.data.data[0] = deg_l;
    msg_arm_degrees.data.data[1] = deg_r;
}

