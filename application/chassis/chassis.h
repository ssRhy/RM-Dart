/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      底盘控制任务所需要的变量和函数
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

#ifndef CHASSIS_H
#define CHASSIS_H

#include "robot_param.h"

#if CHASSIS_TYPE != CHASSIS_NONE

#include "custom_typedef.h"
#include "struct_typedef.h"

#define rc_deadband_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }

typedef enum __ChassisState {
    CHASSIS_STATE_NORNAL,  // 底盘正常状态
    CHASSIS_STATE_ERROR    // 底盘错误状态
} ChassisState_e;

// clang-format off
typedef struct
{
    void     (*SetCali)(void);
    void     (*CmdCali)(void);
    void     (*GetStatus)(void);
    uint32_t (*GetDuration)(void);
    float    (*GetSpeedVx)(void);
    float    (*GetSpeedVy)(void);
    float    (*GetSpeedWz)(void);
} ChassisApi_t;
// clang-format on

extern ChassisApi_t chassis;

extern void GimbalSpeedVectorToChassisSpeedVector(
    ChassisSpeedVector_t * speed_vector_set, float dyaw);

#endif  // CHASSIS_TYPE
#endif  // CHASSIS_H
/*------------------------------ End of File ------------------------------*/
