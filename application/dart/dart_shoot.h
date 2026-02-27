/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_trans.c/h
  * @brief      电机飞镖机构控制器。
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
#ifndef DART_SHOOT_H
#define DART_SHOOT_H
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



// 飞镖模式枚举
typedef enum {
    SHOOT_MOTOR_FIRE,
    SHOOT_MOTOR_STOP,
} ShootMode_e;

typedef struct {
    fp32 motor_speed_fdb_front;       // 电机速度反馈
    fp32 motor_speed_fdb_mid;       // 电机速度反馈
    fp32 motor_speed_fdb_rear;       // 电机速度反馈
} ShootFdb_t;

typedef struct {
    fp32 motor_speed_ref_front;   // 电机速度期望
    fp32 motor_speed_ref_mid;   // 电机速度期望
    fp32 motor_speed_ref_rear;   // 电机速度期望
} ShootRef_t;

// Dart模块主结构体
typedef struct
{
    Motor_s shoot_motor_front[2];     // 电机数组
    Motor_s shoot_motor_mid[2];
    Motor_s shoot_motor_rear[2];
   
    
    /*-------------------- Values --------------------*/
    ShootRef_t motor_ref;             // 期望值
               
    ShootFdb_t motor_fdb;            // 反馈值
    ShootMode_e mode;                 // 当前飞镖模式
    /*-------------------- Controllers --------------------*/
    pid_type_def motor_speed_pid_front;   // 速度PID控制器
    pid_type_def motor_speed_pid_mid;   // 速度PID控制器
    pid_type_def motor_speed_pid_rear;   // 速度PID控制器
    
    
    /*-------------------- Status --------------------*/

    uint32_t time;
    uint32_t last_time;
    uint8_t move_flag;

    int16_t last_ecd; //     上一个ecd
    int16_t ecd_count;//     ecd计数
} Shoot_s;

extern void DartShootInit(void);
extern void DartShootSetMode(void);
extern void DartShootObserver(void);
extern void DartShootReference(void);
extern void DartShootConsole(void);
extern void DartShootSendCmd(void);

#endif  // DART_SHOOT_H
#endif  // CHASSIS_TYPE == DART_CHASSIS
