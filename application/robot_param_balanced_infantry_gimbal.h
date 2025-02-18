/**
  * @file       robot_param_balanced_infantry.h
  * @brief      这里是平衡步兵机器人云台部分参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT                  // 选择云台类型

// clang-format off

/*-------------------- Gimbal --------------------*/
//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (0.5f)
//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY  ( 100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE ( 20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_DIRECT_YAW_ID   ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)2)

//电机can口
#define GIMBAL_DIRECT_YAW_CAN   ((uint8_t)2)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE   ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION   ( 1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (-1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO   (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE   (DJI_VOLTAGE_MODE)
#define GIMBAL_DIRECT_PITCH_MODE (DJI_VOLTAGE_MODE)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH ( 0.33f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.78f)

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (-1.6797f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID   ( M_PI)     //云台初始化正对齐的时候使用的yaw轴正中心量

//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE       (28.0f)
#define KI_GIMBAL_YAW_ANGLE       (0.0f)
#define KD_GIMBAL_YAW_ANGLE       (10.0f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_YAW_ANGLE  (20.0f)
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY       (5000.0f)
#define KI_GIMBAL_YAW_VELOCITY       (10.0f)
#define KD_GIMBAL_YAW_VELOCITY       (0.0f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (2000.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY  (25000.0f)

//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE       (45.0f)
#define KI_GIMBAL_PITCH_ANGLE       (0.0f)
#define KD_GIMBAL_PITCH_ANGLE       (10.0f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE  (20.0f)
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY       (7000.0f)
#define KI_GIMBAL_PITCH_VELOCITY       (15.0f)
#define KD_GIMBAL_PITCH_VELOCITY       (0.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (3000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY  (25000.0f)

// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */
/*------------------------------ End of File ------------------------------*/
