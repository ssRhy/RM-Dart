/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_trans.c/h
  * @brief      飞镖机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-01-25      Assistant       1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "robot_param.h"

#if (CHASSIS_TYPE == DART_CHASSIS)
#ifndef DART_H
#define DART_H

#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_communication.h"
#include "math.h"
#include "usb_debug.h"
#include "user_lib.h"
#include "arm_math.h"
#include "referee.h"
#include "detect_task.h"
#include "cmsis_os.h"

// 飞镖模式枚举
typedef enum {
    MOTOR_ANGEL = 0,
    MOTOR_STOP,
} DartMode_e;

typedef struct {
    fp32 motor_speed_fdb;       // 电机速度反馈
    fp32 motor_angle_fdb;       // 电机角度反馈
    fp32 motor_current_fdb;     // 电机电流反馈
} DartFdb_t;

typedef struct {
    fp32 motor_speed_ref;   // 电机速度期望
    fp32 motor_angle_ref;   // 电机角度期望
    fp32 motor_current_ref; // 电机电流期望
} DartRef_t;

// Dart模块主结构体
typedef struct
{
    Motor_s dart_motor;  
    
    /*-------------------- Values --------------------*/
    DartRef_t motor_ref;             // 期望值
    DartFdb_t motor_fdb;             // 反馈值
    DartMode_e mode;                 // 当前飞镖模式
    /*-------------------- Controllers --------------------*/
    pid_type_def motor_speed_pid;   // 速度PID控制器
    pid_type_def motor_angle_pid;   // 角度PID控制器
    
    /*-------------------- Status --------------------*/
    uint32_t timer;
    uint32_t time;
    uint32_t last_time;
    uint8_t move_flag;
    int16_t last_ecd;       // 上一个ecd
    int16_t ecd_count;      // ecd计数
    fp32 last_motor_angle;  // 上次电机角度记录
} Dart_s;

extern void DartInit(void);
extern void DartObserver(void);
extern void DartSetMode(void);
extern void DartReference(void);
extern void DartConsole(void);
extern void DartSendCmd(void);

#endif /* DART_CHASSIS */
#endif /* DART_H */

