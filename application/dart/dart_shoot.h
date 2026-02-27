/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_shoot.c/h
  * @brief      飞镖射击机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  *             射击机构由两个摩擦轮电机组成，采用速度闭环控制
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-1-24        HY           1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#ifndef DART_SHOOT_H
#define DART_SHOOT_H
#include "robot_param.h"

#if (CHASSIS_TYPE == DART_CHASSIS)

#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_communication.h"
#include "math.h"
#include "usb_debug.h"
#include "supervisory_computer_cmd.h"
#include "user_lib.h"
#include "arm_math.h"
#include "referee.h"
#include "detect_task.h"
#include "cmsis_os.h"

// 射击模式枚举
typedef enum {
    SHOOT_STOP = 0,     // 停止（摩擦轮不转）
    SHOOT_READY,        // 待机转速（低速预热）
    SHOOT_LAUNCH,       // 发射转速（全速）
} LoadMode_e;

typedef struct feedback{
    fp32 shoot_speed_fdb_L;       // 电机速度反馈
    fp32 shoot_speed_fdb_R;     // 电机电流反馈
} Fdb;

typedef struct reference
{
  fp32 shoot_speed_ref_L;   // 摩擦轮速度期望
  fp32 shoot_speed_ref_R;
} Ref;



// 射击模块主结构体
typedef struct
{
    LoadMode_e mode;        // 当前射击模式

    Motor_s shoot_motor_L;  // 左摩擦轮电机
    Motor_s shoot_motor_R;  // 右摩擦轮电机

    // pid
    pid_type_def shoot_speed_pid_L;
    pid_type_def shoot_speed_pid_R;

    // feedback
    Fdb FDB;

    // reference
    Ref REF;

  

    // shoot control flag
    uint16_t shoot_flag;    // 发射指令标志（1=发射，0=待机）
    uint32_t time;          // 当前系统时间
    uint32_t last_time;     // 上次发射时间戳

    // flag
    uint16_t fric_flag;     // 摩擦轮状态

    // ecd
    int16_t last_ecd;       // 上一个ecd
    int16_t ecd_count;      // ecd计数
} Shoot_s;



extern void DartShootInit(void);

extern void DartShootSetMode(void);

extern void DartShootObserver(void);

extern void DartShootReference(void);

extern void DartShootConsole(void);

extern void DartShootSendCmd(void);

#endif  // DART_SHOOT_H
#endif  // CHASSIS_TYPE == DART_CHASSIS
