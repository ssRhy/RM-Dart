/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_chasis.c/h
  * @brief      飞镖底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-10-25      CJH            1. 完成基本框架
  *  V1.1.0     2025-11-08      CJH            1. 完成电机控制任务
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"

#if (CHASSIS_TYPE == DART_CHASSIS)
#ifndef DART_CHASSIS_H
#define DART_CHASSIS_H
#include "struct_typedef.h"
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



// 底盘模式枚举
typedef enum {
    CHASSIS_MOTOR_ANGEL = 0,
    CHASSIS_MOTOR_STOP,
} ChassisMode_e;

typedef struct {
    fp32 motor_speed_fdb;       // 电机速度反馈
    fp32 motor_angle_fdb;       // 电机角度反馈
    fp32 motor_current_fdb;     // 电机电流反馈
} ChassisFdb_t;

typedef struct {
    fp32 motor_speed_ref;   // 电机速度期望
    fp32 motor_angle_ref;   // 电机角度期望
    fp32 motor_current_ref; // 电机电流期望
} ChassisRef_t;

// 底盘模块主结构体
typedef struct
{
    Motor_s chassis_motor;     // 底盘电机
    
    /*-------------------- Values --------------------*/
    ChassisRef_t motor_ref;             // 期望值
    ChassisFdb_t motor_fdb;             // 反馈值
    ChassisMode_e mode;                 // 当前底盘模式
    /*-------------------- Controllers --------------------*/
    pid_type_def motor_speed_pid;   // 速度PID控制器
    pid_type_def motor_angle_pid;   // 角度PID控制器
    
    /*-------------------- Status --------------------*/
    fp32 last_angle;      // 上次电机角度记录

    uint32_t time;
    uint32_t last_time;
    uint8_t move_flag;

    int16_t last_ecd; //     上一个ecd
    int16_t ecd_count;//     ecd计数
} Chassis_s;

extern void ChassisInit(void);
extern void ChassisSetMode(void);
extern void ChassisObserver(void);
extern void ChassisReference(void);
extern void ChassisConsole(void);
extern void ChassisSendCmd(void);

#endif  // DART_CHASSIS_H
#endif  // CHASSIS_TYPE == DART_CHASSIS
