#ifndef IMU_H
#define IMU_H

#include "struct_typedef.h"
#include "macro_typedef.h"

extern inline float GetImuAngle(uint8_t axis);
extern inline float GetImuVelocity(uint8_t axis);
extern inline float GetImuAccel(uint8_t axis);
extern float GetYawBias(void);

#endif  // IMU_H
/*------------------------------ End of File ------------------------------*/
