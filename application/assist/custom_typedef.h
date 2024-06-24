#ifndef __CUSTOM_TYPEDEF_H
#define __CUSTOM_TYPEDEF_H

typedef struct __Imu
{
    float yaw, pitch, roll;              // rad
    float yaw_vel, pitch_vel, roll_vel;  // rad/s
    float x_accel, y_accel, z_accel;     // m/s^2
} Imu_t;

typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;

#endif  // __CUSTOM_TYPEDEF_H
