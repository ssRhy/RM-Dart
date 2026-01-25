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
#ifndef DART_FEED_H
#define DART_FEED_H
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



// 供弹模式枚举
typedef enum {
    FEED_MOTOR_ANGEL = 0,
    FEED_MOTOR_STOP,
} FeedMode_e;

typedef struct {
    fp32 motor_speed_fdb;       // 电机速度反馈
    fp32 motor_angle_fdb;       // 电机角度反馈
    fp32 motor_current_fdb;     // 电机电流反馈
} FeedFdb_t;

typedef struct {
    fp32 motor_speed_ref;   // 电机速度期望
    fp32 motor_angle_ref;   // 电机角度期望
    fp32 motor_current_ref; // 电机电流期望
} FeedRef_t;

// Feed模块主结构体
typedef struct
{
    Motor_s feed_motor;     // 电机数组
    
    /*-------------------- Values --------------------*/
    FeedRef_t motor_ref;             // 期望值
    FeedFdb_t motor_fdb;             // 反馈值
    FeedMode_e mode;                 // 当前飞镖模式
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
} Feed_s;

extern void DartFeedInit(void);
extern void DartFeedSetMode(void);
extern void DartFeedObserver(void);
extern void DartFeedReference(void);
extern void DartFeedConsole(void);
extern void DartFeedSendCmd(void);

#endif  // DART_FEED_H
#endif  // CHASSIS_TYPE == DART_CHASSIS
