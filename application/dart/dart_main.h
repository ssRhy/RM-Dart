/**
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
 * @file       dart_main.c/h
 * @brief      飞镖主控板机构控制器（chassis + feed + trans 合并管理）
 * @note       参照 shoot_fric_trigger 模块风格，将三个机构统一在一个结构体中，
 *             时序约束：chassis 到位 → feed 到位 → trans 执行
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025-2-27       HY            1. 由 dart_trans + dart_feed 重构合并
 *
 @verbatim
 ==============================================================================

 ==============================================================================
 @endverbatim
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"

#if (CHASSIS_TYPE == DART_CHASSIS) && (DART_BOARD_TYPE == DART_BOARD_MAIN)
#ifndef DART_MAIN_H
#define DART_MAIN_H

#include "motor.h"
#include "pid.h"
#include "CAN_communication.h"
#include "math.h"
#include "usb_debug.h"
#include "user_lib.h"
#include "arm_math.h"
#include "cmsis_os.h"

/* ==================== Chassis 模式枚举 ==================== */
typedef enum {
    CHASSIS_ANGEL = 0,
    CHASSIS_STOP,
} ChassisMode_e;

/* ==================== Trans 模式枚举 ==================== */
typedef enum {
    TRANS_ANGEL = 0,
    TRANS_STOP,
} TransMode_e;

/* ==================== Feed 模式枚举 ==================== */
typedef enum {
    FEED_ANGEL = 0,
    FEED_STOP,
} FeedMode_e;

/* ==================== 通用反馈/期望结构 ==================== */
typedef struct {
    fp32 speed_fdb;
    fp32 angle_fdb;
} MotorFdb_t;

typedef struct {
    fp32 speed_ref;
    fp32 angle_ref;
} MotorRef_t;

/* ==================== 主控板统一结构体 ==================== */
typedef struct
{
    /* ---------- Chassis 电机（ID1，CAN1+0x1FF，DJI_M6020） ---------- */
    Motor_s      chassis_motor;
    MotorRef_t   chassis_ref;
    MotorFdb_t   chassis_fdb;
    ChassisMode_e chassis_mode;
    pid_type_def chassis_speed_pid;
    pid_type_def chassis_angle_pid;
    uint8_t  chassis_move_flag;
    uint8_t  chassis_done_flag;
    int16_t  chassis_last_ecd;
    int16_t  chassis_ecd_count;
    uint32_t chassis_time;
    uint32_t chassis_last_time;

    /* ---------- Feed 电机（ID2，CAN1+0x200，DJI_M2006） ---------- */
    Motor_s      feed_motor;
    MotorRef_t   feed_ref;
    MotorFdb_t   feed_fdb;
    FeedMode_e   feed_mode;
    pid_type_def feed_speed_pid;
    pid_type_def feed_angle_pid;
    uint8_t  feed_move_flag;
    uint8_t  feed_done_flag;
    uint8_t  feed_step;          // 0=未开始, 1=第一段5PI, 2=延时中, 3=第二段5PI
    uint32_t feed_delay_start;   // 第一段到位后记录延时起始 tick
    int16_t  feed_last_ecd;
    int32_t  feed_ecd_count;
    uint32_t feed_time;
    uint32_t feed_last_time;

    /* ---------- Trans 电机（ID3，CAN1+0x200，DJI_M3508） ---------- */
    Motor_s      trans_motor;
    MotorRef_t   trans_ref;
    MotorFdb_t   trans_fdb;
    TransMode_e  trans_mode;
    pid_type_def trans_speed_pid;
    pid_type_def trans_angle_pid;
    uint8_t  trans_move_flag;
    uint8_t  trans_done_flag;
    int16_t  trans_last_ecd;
    int16_t  trans_ecd_count;
    uint32_t trans_time;
    uint32_t trans_last_time;
} DartMain_s;

/* ==================== 对外接口 ==================== */
extern void DartMainInit(void);
extern void DartMainSetMode(void);
extern void DartMainObserver(void);
extern void DartMainReference(void);
extern void DartMainConsole(void);
extern void DartMainSendCmd(void);

#endif  // DART_MAIN_H
#endif  // CHASSIS_TYPE == DART_CHASSIS && DART_BOARD_TYPE == DART_BOARD_MAIN
