/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal.c/h
  * @brief      云台控制任务所需要的变量和函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef GIMBAL_H
#define GIMBAL_H

#include "robot_param.h"

#if GIMBAL_TYPE != GIMBAL_NONE

#include "struct_typedef.h"

// 云台任务相关宏定义
#define GIMBAL_TASK_INIT_TIME 201  // 任务初始化 空闲一段时间
#define GIMBAL_CONTROL_TIME 1      // 云台任务控制间隔 1ms

// 云台的遥控器相关宏定义
#define GIMBAL_YAW_CHANNEL 2    // yaw控制通道
#define GIMBAL_PITCH_CHANNEL 3  // pitch控制通道
#define GIMBAL_MODE_CHANNEL 0   // 状态开关通道
#define GIMBAL_RC_DEADBAND 10   // 摇杆死区

// clang-format off
typedef struct
{
    void     (*SetCali)(void);
    void     (*CmdCali)(void);
    void     (*GetStatus)(void);
    uint32_t (*GetDuration)(void);
    float    (*GetYawMid)(void);
} GimbalApi_t;
// clang-format on

extern GimbalApi_t gimbal;

// API

extern inline uint8_t GetGimbalStatus(void);
extern inline uint32_t GetGimbalDuration(void);
extern inline float GetGimbalSpeed(uint8_t axis);
extern inline float GetGimbalVelocity(uint8_t axis);
extern inline float GetGimbalDeltaYawMid(void);

#endif  // GIMBAL_TYPE
#endif  // GIMBAL_H
/*------------------------------ End of File ------------------------------*/
