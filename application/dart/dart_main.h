/**
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
 * @file       dart_main.c/h
 * @brief      飞镖主控板机构控制器（trans + feed 合并管理）
 * @note       参照 shoot_fric_trigger 模块风格，将 trans 和 feed 统一在一个结构体中，
 *             实现先 trans 到位、再 feed 动作的时序控制，并合并 CAN 帧发送。
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

/* ==================== Trans 反馈/期望 ==================== */
typedef struct {
    fp32 speed_fdb;
    fp32 angle_fdb;
} TransFdb_t;

typedef struct {
    fp32 speed_ref;
    fp32 angle_ref;
} TransRef_t;

/* ==================== Feed 反馈/期望 ==================== */
typedef struct {
    fp32 speed_fdb;
    fp32 angle_fdb;
} FeedFdb_t;

typedef struct {
    fp32 speed_ref;
    fp32 angle_ref;
} FeedRef_t;

/* ==================== 主控板统一结构体 ==================== */
typedef struct
{
    /* ---------- Trans 电机 ---------- */
    Motor_s trans_motor;
    TransRef_t trans_ref;
    TransFdb_t trans_fdb;
    TransMode_e trans_mode;
    pid_type_def trans_speed_pid;
    pid_type_def trans_angle_pid;
    uint8_t  trans_move_flag;
    int16_t  trans_last_ecd;
    int16_t  trans_ecd_count;
    uint32_t trans_time;
    uint32_t trans_last_time;

    /* ---------- Feed 电机 ---------- */
    Motor_s feed_motor;
    FeedRef_t feed_ref;
    FeedFdb_t feed_fdb;
    FeedMode_e feed_mode;
    pid_type_def feed_speed_pid;
    pid_type_def feed_angle_pid;
    uint8_t  feed_move_flag;
    int16_t  feed_last_ecd;
    int16_t  feed_ecd_count;
    uint32_t feed_time;
    uint32_t feed_last_time;
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
